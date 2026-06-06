#include "state_machine.h"
#include "modbus_app.h"
#include "modbus.h"
#include "joystick.h"
#include "pid_control.h"
#include <math.h>
#include <string.h>

extern ModbusSlave_t mb_slave;
extern void Send_Telemetry(void);

/* ── State-machine locals ────────────────────────────────────────────────── */
static SeqMBState_t  seq_mb_state   = SEQ_MB_IDLE;
static uint8_t       seq_mb_pair_idx = 0;
static uint32_t      seq_mb_timer    = 0;
static uint8_t       seq_mb_step     = 0;

static TestState_t   test_state      = TEST_IDLE;
static int16_t       test_repeat     = 0;
static uint8_t       test_mv_armed   = 0;

static JoySeqState_t joy_seq_state   = JSEQ_IDLE;

static ActSeqState_t act_seq_state   = ACT_IDLE;
static uint32_t      act_seq_timer   = 0;
static uint32_t      act_seq_timeout = 50;

/* ── EEPROM stubs ────────────────────────────────────────────────────────── */
static void eeprom_save(float angle_deg) { (void)angle_deg; }

/* ── Forward declarations ────────────────────────────────────────────────── */
static void start_move_deg(float deg);
static void start_move_hole(int16_t raw);

/* ── Homing EXTI control ─────────────────────────────────────────────────── */
void homing_exti_enable(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(HOME_SENSOR_Pin);
  homing_sensor_flag    = 0;
  homing_sensor_pending = 0;
  GPIO_InitTypeDef g    = {0};
  g.Pin  = HOME_SENSOR_Pin;
  g.Mode = GPIO_MODE_IT_RISING;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_SENSOR_GPIO_Port, &g);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void homing_exti_disable(void)
{
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(HOME_SENSOR_Pin);
  GPIO_InitTypeDef g = {0};
  g.Pin  = HOME_SENSOR_Pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_SENSOR_GPIO_Port, &g);
}

void finish_homing(void)
{
  homing_exti_disable();
  homing_sensor_flag = 0;
  __disable_irq();
  enc_reset_pending    = 1;
  cumulative_angle_deg = -home_offset_deg;
  ctrl_direct_target   = 0.0f;
  ctrl_traj_start      = 0.0f;
  motor_speed_cmd      = 0.0f;
  __enable_irq();
  control_reset();
  homed = 1;
  eeprom_save(0.0f);
  mb_slave.registers[0x27] = 0;
  current_state = STATE_IDLE;
}

static void start_move_deg(float deg)
{
  float target_rad   = deg * (3.14159265f / 180.0f);
  ctrl_direct_target = target_rad;
  ctrl_traj_start    = my_encoder.position_rad;
  control_set_target(target_rad);
}

static void start_move_hole(int16_t raw)
{
  int16_t hole       = (raw >= 0) ? raw : (int16_t)(-raw);
  float   target_abs = (float)hole * 5.0f;
  float   pos_cumul  = my_encoder.position_rad * (180.0f / 3.14159265f);
  float   pos_mod    = fmodf(pos_cumul, 360.0f);
  if (pos_mod < 0.0f) pos_mod += 360.0f;
  float disp;
  if (raw >= 0)
    disp =  fmodf((target_abs - pos_mod) + 360.0f, 360.0f);
  else
    disp = -fmodf((pos_mod - target_abs) + 360.0f, 360.0f);
  start_move_deg(pos_cumul + disp);
}

/* ── Main 100Hz tick ─────────────────────────────────────────────────────── */
void state_machine_tick(void)
{
  /* ── Joystick UART health ────────────────────────────────────────────── */
  extern uint8_t joy_dma_buf[];
  {
    static uint8_t joy_was_alive = 0;
    uint8_t joy_now_alive = joy_rp2040_alive();
    if (joy_was_alive && !joy_now_alive) {
      HAL_UART_AbortReceive(&huart3);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart3, joy_dma_buf, PKT_LEN * 2);
      __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
    joy_was_alive = joy_now_alive;
  }

  /* ── EMER block ──────────────────────────────────────────────────────── */
  float   safe_speed  = 0.0f;
  uint8_t emer_active = (current_state == STATE_EMER);
  if (emer_active) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
    mb_slave.registers[0x27] = 0;
    motor_speed_cmd = 0.0f;

    static uint8_t emer_rel_cnt = 0;
    uint8_t estop_released = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port,     ESTOP_Pin)     == GPIO_PIN_SET);
    uint8_t reset_pressed  = (HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin) == GPIO_PIN_RESET);
    if (estop_released && reset_pressed) {
      if (emer_rel_cnt < 10) emer_rel_cnt++;
    } else {
      emer_rel_cnt = 0;
    }
    if (emer_rel_cnt >= 5) {
      emer_rel_cnt = 0;
      dev_dash.Cmd.start_move  = 0;
      dev_dash.Cmd.cancel_move = 0;
      float hold_pos           = current_position * RAD_PER_CNT;
      ctrl_direct_target       = hold_pos;
      ctrl_traj_start          = hold_pos;
      dev_dash.Cmd.target_deg  = hold_pos * (180.0f / 3.14159f);
      mb_slave.registers[0x24] = 0;
      mb_slave.registers[0x01] = 0;
      mb_slave.registers[0x05] = 0;
      mb_slave.registers[0x25] = 0;
      control_reset();
      motor_speed_cmd = 0.0f;
      HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);
      if (dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {
        GPIO_PinState mode_now = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
        current_state = (mode_now == GPIO_PIN_RESET) ? STATE_IDLE : STATE_MANUAL;
      } else {
        current_state = STATE_IDLE;
      }
    }
  }

  /* ── SET HOME: RT + A (any non-EMER state) ───────────────────────────── */
  if (!emer_active) {
    if (joy_is_connected() && joy_rt_f() > 0.5f && joy_btn(BTN_A)) {
      dev_dash.Cmd.set_home = 1;
    }
    if (dev_dash.Cmd.set_home) {
      dev_dash.Cmd.set_home = 0;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      motor_speed_cmd      = 0.0f;
      home_offset_deg      = cumulative_angle_deg;
      __disable_irq();
      enc_reset_pending    = 1;
      cumulative_angle_deg = 0.0f;
      ctrl_direct_target   = 0.0f;
      ctrl_traj_start      = 0.0f;
      __enable_irq();
      control_reset();
      control_set_target(0.0f);
      homed           = 1;
      sethome_led_cnt = 1;
    }
  }

  /* ── System Mode Selector ────────────────────────────────────────────── */
  if (!emer_active) {

    if (dev_dash.Sys.mode == SYS_MODE_HARDWARE_TEST) {

      HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,   PNEUMATIC_Pin,   dev_dash.Test.force_pneumatic ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GRIPPER_GPIO_Port,     GRIPPER_Pin,     dev_dash.Test.force_gripper   ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TOWER_G_GPIO_Port,     TOWER_G_Pin,     dev_dash.Test.force_tower_g   ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TOWER_Y_GPIO_Port,     TOWER_Y_Pin,     dev_dash.Test.force_tower_y   ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TOWER_R_GPIO_Port,     TOWER_R_Pin,     dev_dash.Test.force_tower_r   ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, dev_dash.Test.force_emer      ? GPIO_PIN_RESET : GPIO_PIN_SET);

      safe_speed = dev_dash.Test.force_motor;
      if (safe_speed >  1.0f) safe_speed =  1.0f;
      if (safe_speed < -1.0f) safe_speed = -1.0f;
      if (safe_speed >= 0.0f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        safe_speed = -safe_speed;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

    } else if (dev_dash.Sys.mode == SYS_MODE_JOYSTICK_TEST) {

      typedef enum {
        SEQ_IDLE = 0,
        SEQ_PICK_OPEN_WAIT,
        SEQ_PICK_WAIT1,
        SEQ_PICK_WAIT2,
        SEQ_PLACE_WAIT1,
        SEQ_PLACE_WAIT2,
      } SeqState_t;
      static SeqState_t seq_state    = SEQ_IDLE;
      static uint32_t   seq_timer    = 0;
      static uint8_t    lb_prev      = 0;
      static uint8_t    rb_prev      = 0;
      static uint8_t    y_prev       = 0;
      static uint8_t    gripper_latch = 0;

      if (joy_is_connected()) {
        uint32_t seq_ticks = (uint32_t)(seq_delay_s * 100.0f);
        if (seq_ticks < 1) seq_ticks = 1;

        switch (seq_state) {
          case SEQ_IDLE:
            if (joy_btn(BTN_LB) && !lb_prev) {
              gripper_latch = 0;
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
              seq_timer = 0;
              seq_state = SEQ_PICK_OPEN_WAIT;
            } else if (joy_btn(BTN_RB) && !rb_prev) {
              HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);
              seq_timer = 0;
              seq_state = SEQ_PLACE_WAIT1;
            }
            break;
          case SEQ_PICK_OPEN_WAIT:
            if (++seq_timer >= seq_ticks) {
              HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);
              seq_timer = 0; seq_state = SEQ_PICK_WAIT1;
            }
            break;
          case SEQ_PICK_WAIT1:
            if (++seq_timer >= seq_ticks) {
              gripper_latch = 1;
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);
              seq_timer = 0; seq_state = SEQ_PICK_WAIT2;
            }
            break;
          case SEQ_PICK_WAIT2:
            if (++seq_timer >= seq_ticks) {
              HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET);
              seq_state = SEQ_IDLE;
            }
            break;
          case SEQ_PLACE_WAIT1:
            if (++seq_timer >= seq_ticks) {
              gripper_latch = 0;
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
              seq_timer = 0; seq_state = SEQ_PLACE_WAIT2;
            }
            break;
          case SEQ_PLACE_WAIT2:
            if (++seq_timer >= seq_ticks) {
              HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET);
              seq_state = SEQ_IDLE;
            }
            break;
          default:
            seq_state = SEQ_IDLE;
            break;
        }

        lb_prev = joy_btn(BTN_LB);
        rb_prev = joy_btn(BTN_RB);

        if (seq_state == SEQ_IDLE) {
          HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, joy_btn(BTN_B) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,   PNEUMATIC_Pin,   joy_btn(BTN_X) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          if (joy_btn(BTN_Y) && !y_prev) {
            gripper_latch ^= 1;
            HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, gripper_latch ? GPIO_PIN_SET : GPIO_PIN_RESET);
          }
        }
        y_prev = joy_btn(BTN_Y);

        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, joy_btn(BTN_A) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, joy_btn(BTN_DPAD_UP)   ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, joy_btn(BTN_DPAD_DOWN) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        uint8_t fine_left  = joy_btn(BTN_DPAD_LEFT);
        uint8_t fine_right = joy_btn(BTN_DPAD_RIGHT);

        static float   joy_ramp     = 0.0f;
        static uint8_t joy_dead_cnt = 0;

        if (fine_left || fine_right) {
          float ft = homing_slow_speed;
          if (ft < 0.01f) ft = 0.01f;
          if (ft > 1.0f)  ft = 1.0f;
          safe_speed = fine_left ? ft : -ft;
          joy_ramp   = 0.0f;
        } else {
          float joy_raw    = (joy_rt_f() > 0.5f) ? joy_ly_f() : 0.0f;
          float joy_target = (joy_raw >  dev_dash.Sys.max_speed) ?  dev_dash.Sys.max_speed :
                             (joy_raw < -dev_dash.Sys.max_speed) ? -dev_dash.Sys.max_speed : joy_raw;
          float ramp_rate  = dev_dash.Sys.ramp_rate;
          if ((joy_target > 0.0f && joy_ramp < 0.0f) ||
              (joy_target < 0.0f && joy_ramp > 0.0f)) {
            joy_dead_cnt = 5;
          }
          if (joy_dead_cnt > 0) { joy_ramp = 0.0f; joy_dead_cnt--; }
          else if (joy_target > joy_ramp + ramp_rate) joy_ramp += ramp_rate;
          else if (joy_target < joy_ramp - ramp_rate) joy_ramp -= ramp_rate;
          else                                         joy_ramp  = joy_target;
          safe_speed = joy_ramp;
        }

        if (safe_speed >= 0.0f) {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
          safe_speed = -safe_speed;
        }
        if (safe_speed > 1.0f) safe_speed = 1.0f;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else {
        seq_state = SEQ_IDLE;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRIPPER_GPIO_Port,     GRIPPER_Pin,     GPIO_PIN_RESET);
      }

    } else if (dev_dash.Sys.mode == SYS_MODE_AUTO_MOTOR_TEST) {

      static uint16_t auto_test_timer = 0;
      static uint8_t  auto_test_dir   = 0;
      static float    auto_ramp       = 0.0f;
      static uint8_t  auto_dead_cnt   = 0;

      uint16_t cur_period_ms = auto_test_dir ? dev_dash.Test.test_period_fwd_ms
                                             : dev_dash.Test.test_period_rev_ms;
      uint16_t half_period = (cur_period_ms / 10);
      if (half_period < 1) half_period = 1;
      auto_test_timer++;
      if (auto_test_timer >= half_period) {
        auto_test_timer = 0;
        auto_test_dir   = !auto_test_dir;
        auto_dead_cnt   = 5;
      }
      float auto_target = dev_dash.Test.test_speed;
      if (auto_target > dev_dash.Sys.max_speed) auto_target = dev_dash.Sys.max_speed;
      if (auto_target < 0.0f) auto_target = 0.0f;

      if (auto_dead_cnt > 0) { auto_ramp = 0.0f; auto_dead_cnt--; }
      else if (auto_target > auto_ramp + dev_dash.Sys.ramp_rate) auto_ramp += dev_dash.Sys.ramp_rate;
      else if (auto_target < auto_ramp - dev_dash.Sys.ramp_rate) auto_ramp -= dev_dash.Sys.ramp_rate;
      else                                                         auto_ramp  = auto_target;

      safe_speed = auto_ramp;
      HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                        auto_test_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

    } else {

      /* ── PRODUCTION MODE ──────────────────────────────────────────────── */
      static RobotState_t prev_state = STATE_INIT;
      uint8_t entering_pid_state = (
        (current_state == STATE_AUTO     && prev_state != STATE_AUTO)     ||
        (current_state == STATE_SEQUENCE && prev_state != STATE_SEQUENCE) ||
        (current_state == STATE_TEST     && prev_state != STATE_TEST)
      );
      if (entering_pid_state && !homing_final_zero_pending) {
        float pos_now      = current_position * RAD_PER_CNT;
        ctrl_direct_target = pos_now;
        ctrl_traj_start    = pos_now;
        if (!dev_dash.Cmd.start_move)
          dev_dash.Cmd.target_deg = pos_now * (180.0f / 3.14159f);
        control_reset();
        control_set_target(pos_now);
        mb_slave.registers[0x05] = 0;
      }
      prev_state = current_state;

      /* Post-homing proximity offset: re-zero encoder once arm settles */
      if (homing_final_zero_pending && ctrl_settled) {
        homing_final_zero_pending = 0;
        if (homing_rezero_on_arrive) {
          homing_rezero_on_arrive = 0;
          __disable_irq();
          enc_reset_pending    = 1;
          cumulative_angle_deg = 0.0f;
          ctrl_direct_target   = 0.0f;
          ctrl_traj_start      = 0.0f;
          __enable_irq();
          control_reset();
          control_set_target(0.0f);
          home_offset_deg = 0.0f;
        }
        current_state = STATE_IDLE;
      }

      /* LT+LB: HOME via proximity sensor */
      if (joy_is_connected() && !emer_active) {
        static uint8_t ltlb_prev_g = 0;
        uint8_t ltlb_g = (joy_lt_f() > 0.5f && joy_btn(BTN_LB));
        if (ltlb_g && !ltlb_prev_g &&
            current_state != STATE_HOMING_FAST &&
            current_state != STATE_HOMING_BACKOFF &&
            current_state != STATE_HOMING_SLOW) {
          motor_speed_cmd = 0.0f;
          control_reset();
          homing_exti_enable();
          current_state = STATE_HOMING_FAST;
        }
        ltlb_prev_g = ltlb_g;
      }

      /* GO HOME: LT + A → 0° (SET HOME) */
      if (joy_is_connected() && !emer_active) {
        static uint8_t lta_prev_g = 0;
        uint8_t lta_g = (joy_lt_f() > 0.5f && joy_btn(BTN_A));
        if (lta_g && !lta_prev_g && homed &&
            (current_state == STATE_IDLE   || current_state == STATE_AUTO ||
             current_state == STATE_MANUAL || current_state == STATE_MANUAL_MB)) {
          homing_final_zero_pending = 0;
          homing_rezero_on_arrive   = 0;
          skip_p2p_entry = 1;
          current_state  = STATE_AUTO;
          start_move_deg(0.0f);
        }
        lta_prev_g = lta_g;
      }

      /* LT + B → 0° shortest path */
      if (joy_is_connected() && joy_lt_f() > 0.5f && joy_btn(BTN_B)) {
        if (current_state == STATE_IDLE    || current_state == STATE_MANUAL ||
            current_state == STATE_MANUAL_MB || current_state == STATE_AUTO) {
          dev_dash.Cmd.target_deg = 0.0f;
          dev_dash.Cmd.start_move = 1;
          skip_p2p_entry = 1;
          current_state  = STATE_AUTO;
        }
      }

      /* reg[0x05] jog */
      {
        int16_t jog_g = (int16_t)mb_slave.registers[0x05];
        if (entering_pid_state) mb_slave.registers[0x05] = 0;
        if (jog_g != 0 && !entering_pid_state &&
            current_state != STATE_EMER &&
            current_state != STATE_HOMING_FAST &&
            current_state != STATE_HOMING_BACKOFF &&
            current_state != STATE_HOMING_SLOW) {
          float pos_now    = my_encoder.position_rad;
          float jog_rad    = (float)jog_g * (3.14159265f / 180.0f);
          float target_rad = pos_now + jog_rad;
          ctrl_traj_start    = pos_now;
          ctrl_direct_target = target_rad;
          control_set_target(target_rad);
          skip_p2p_entry = 1;
          current_state  = STATE_AUTO;
          mb_slave.registers[0x27] = 0x08;
          mb_slave.registers[0x05] = 0;
        }
      }

      /* reg[0x01] mode commands */
      if (current_state != STATE_EMER &&
          current_state != STATE_HOMING_FAST &&
          current_state != STATE_HOMING_BACKOFF &&
          current_state != STATE_HOMING_SLOW) {
        uint16_t mode_cmd = mb_slave.registers[0x01];
        if (mode_cmd & 0x01) {
          if (homed) {
            homing_final_zero_pending = 0;
            homing_rezero_on_arrive   = 0;
            skip_p2p_entry = 1;
            current_state  = STATE_AUTO;
            start_move_deg(0.0f);
          } else {
            motor_speed_cmd = 0.0f;
            control_reset();
            homing_exti_enable();
            current_state = STATE_HOMING_FAST;
          }
          mb_slave.registers[0x01] = 0;
        } else if (mode_cmd & 0x02) {
          control_reset();
          motor_speed_cmd = 0.0f;
          if (homed && mb_slave.registers[0x22] > 0 && (mb_slave.registers[0x04] & 0x01)) {
            seq_mb_state    = SEQ_MB_IDLE;
            seq_mb_pair_idx = 0;
            seq_mb_timer    = 0;
            seq_mb_step     = 0;
            mb_slave.registers[0x04] &= ~0x01u;
            current_state   = STATE_SEQUENCE;
          } else {
            current_state   = STATE_MANUAL_MB;
          }
          mb_slave.registers[0x01] = 0;
        } else if (mode_cmd & 0x04) {
          if (homed) { skip_p2p_entry = 0; current_state = STATE_AUTO; seq_mb_state = SEQ_MB_IDLE; seq_mb_pair_idx = 0; }
          mb_slave.registers[0x01] = 0;
        } else if (mode_cmd & 0x08) {
          dev_dash.Cmd.set_home = 1;
          mb_slave.registers[0x01] = 0;
        } else if (mode_cmd & 0x10) {
          if (homed) { current_state = STATE_TEST; test_state = TEST_GOING_END; test_mv_armed = 1; test_repeat = (int16_t)mb_slave.registers[0x11]; }
          mb_slave.registers[0x01] = 0;
        }
      }

      /* Cancel / EMER via dev_dash */
      if (dev_dash.Cmd.cancel_move) {
        dev_dash.Cmd.cancel_move = 0;
        motor_speed_cmd          = 0.0f;
        control_reset();
        __disable_irq();
        current_state = STATE_EMER;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        __enable_irq();
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
      }

      /* X button: EMER */
      if (joy_is_connected() && joy_btn(BTN_X) && current_state != STATE_EMER) {
        motor_speed_cmd = 0.0f;
        control_reset();
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
        current_state = STATE_EMER;
      }

      /* ── State machine switch ──────────────────────────────────────────── */
      switch (current_state) {

        case STATE_IDLE:
          mb_slave.registers[0x27] = 0;
          if (joy_is_connected()) {
            if (joy_rt_f() > 0.5f) {
              motor_speed_cmd = joy_ly_f();
            } else if (joy_btn(BTN_DPAD_LEFT)) {
              motor_speed_cmd =  (homing_slow_speed > 0.01f ? homing_slow_speed : 0.01f);
            } else if (joy_btn(BTN_DPAD_RIGHT)) {
              motor_speed_cmd = -(homing_slow_speed > 0.01f ? homing_slow_speed : 0.01f);
            } else {
              motor_speed_cmd = 0.0f;
            }
            if (joy_lt_f() > 0.5f && joy_btn(BTN_LB)) { homing_exti_enable(); current_state = STATE_HOMING_FAST; }
            static uint8_t t_prev_idle = 0;
            if (joy_btn(BTN_T) && !t_prev_idle && homed) current_state = STATE_AUTO;
            t_prev_idle = joy_btn(BTN_T);
          } else {
            motor_speed_cmd = 0.0f;
            joy_seq_state   = JSEQ_IDLE;
          }
          break;

        case STATE_HOMING_FAST:
          mb_slave.registers[0x27] = 0x01;
          motor_speed_cmd = -homing_fast_speed;
          if (homing_sensor_flag) {
            homing_sensor_flag = 0;
            motor_speed_cmd    = 0.0f;
            current_state      = STATE_HOMING_BACKOFF;
          }
          break;

        case STATE_HOMING_BACKOFF: {
          mb_slave.registers[0x27] = 0x01;
          static uint32_t back_tick = 0;
          motor_speed_cmd = homing_backoff_speed;
          if (++back_tick >= homing_backoff_ticks) {
            back_tick       = 0;
            motor_speed_cmd = 0.0f;
            homing_exti_enable();
            current_state   = STATE_HOMING_SLOW;
          }
          break;
        }

        case STATE_HOMING_SLOW:
          mb_slave.registers[0x27] = 0x01;
          motor_speed_cmd = -homing_slow_speed;
          if (homing_sensor_flag) {
            homing_sensor_flag = 0;
            motor_speed_cmd    = 0.0f;
            finish_homing();
          }
          break;

        case STATE_MANUAL: {
          mb_slave.registers[0x27] = 0;
          static float   man_ramp    = 0.0f;
          static uint8_t man_cyl     = 0;
          static uint8_t man_grip    = 0;
          static uint8_t y_prev_man  = 0, b_prev_man = 0;
          static uint8_t lb_prev_man = 0, rb_prev_man = 0;

          if (joy_is_connected()) {
            uint8_t rt_held = (joy_rt_f() > 0.5f);
            uint8_t btn_y   = joy_btn(BTN_Y);
            uint8_t btn_b   = joy_btn(BTN_B);
            uint8_t btn_lb  = joy_btn(BTN_LB);
            uint8_t btn_rb  = joy_btn(BTN_RB);

            float tgt = rt_held ? (joy_ly_f() * 0.6f) : 0.0f;
            if (fabsf(tgt) < 0.08f) tgt = 0.0f;
            const float MAN_RAMP = 0.03f;
            if      (tgt > man_ramp + MAN_RAMP) man_ramp += MAN_RAMP;
            else if (tgt < man_ramp - MAN_RAMP) man_ramp -= MAN_RAMP;
            else                                 man_ramp  = tgt;
            motor_speed_cmd = man_ramp;

            if      (joy_btn(BTN_DPAD_RIGHT)) motor_speed_cmd =  homing_slow_speed;
            else if (joy_btn(BTN_DPAD_LEFT))  motor_speed_cmd = -homing_slow_speed;

            if (btn_y && !y_prev_man) {
              man_cyl ^= 1;
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, man_cyl ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            if (btn_b && !b_prev_man) {
              man_grip ^= 1;
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, man_grip ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            if (joy_lt_f() > 0.5f && btn_lb && !lb_prev_man) {
              man_ramp        = 0.0f;
              motor_speed_cmd = 0.0f;
              control_reset();
              homing_exti_enable();
              current_state = STATE_HOMING_FAST;
              lb_prev_man = btn_lb;
              break;
            }
            if (btn_lb && !lb_prev_man && act_seq_state == ACT_IDLE) {
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port,   GRIPPER_Pin,   GPIO_PIN_RESET);
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
              act_seq_state = ACT_PK_DOWN;
              act_seq_timer = 0;
            }
            if (btn_rb && !rb_prev_man && act_seq_state == ACT_IDLE) {
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
              act_seq_state = ACT_PL_DOWN;
              act_seq_timer = 0;
            }
            y_prev_man  = btn_y;
            b_prev_man  = btn_b;
            lb_prev_man = btn_lb;
            rb_prev_man = btn_rb;
          } else {
            if      (man_ramp >  0.01f) man_ramp -= 0.05f;
            else if (man_ramp < -0.01f) man_ramp += 0.05f;
            else                        man_ramp  = 0.0f;
            motor_speed_cmd = man_ramp;
          }
          break;
        }

        case STATE_MANUAL_MB:
          mb_slave.registers[0x27] = 0;
          motor_speed_cmd = 0.0f;
          break;

        case STATE_AUTO: {
          uint8_t traj_done = ctrl_settled;

          if (mb_slave.registers[0x22] > 0 && (mb_slave.registers[0x04] & 0x01)) {
            seq_mb_state    = SEQ_MB_IDLE;
            seq_mb_pair_idx = 0;
            seq_mb_timer    = 0;
            seq_mb_step     = 0;
            mb_slave.registers[0x04] &= ~0x01u;
            current_state   = STATE_SEQUENCE;
            break;
          }

          mb_slave.registers[0x27] = 0x08;
          {
            int16_t p2p_raw = (int16_t)mb_slave.registers[0x24];
            static int16_t last_p2p_raw = -32768;
            if (entering_pid_state && !homing_final_zero_pending) last_p2p_raw = p2p_raw;
            if (skip_p2p_entry) {
              last_p2p_raw   = p2p_raw;
              skip_p2p_entry = 0;
            } else if (dev_dash.Cmd.start_move) {
              dev_dash.Cmd.start_move = 0;
              start_move_deg(dev_dash.Cmd.target_deg);
            } else if (p2p_raw != last_p2p_raw) {
              last_p2p_raw = p2p_raw;
              start_move_deg((float)p2p_raw);
            }
          }
          if (traj_done) mb_slave.registers[0x27] = 0;

          if (mb_slave.registers[0x25] & 0x01) {
            control_reset();
            current_state = STATE_IDLE;
            mb_slave.registers[0x25] = 0;
            mb_slave.registers[0x27] = 0;
          }

          if (joy_is_connected()) {
            static uint8_t t_prev_auto = 0;
            if (joy_btn(BTN_T) && !t_prev_auto) {
              float hold = my_encoder.position_rad;
              ctrl_direct_target      = hold;
              dev_dash.Cmd.target_deg = hold * (180.0f / 3.14159265f);
              control_reset();
              current_state = STATE_IDLE;
            }
            t_prev_auto = joy_btn(BTN_T);
            static uint8_t dl_prev = 0, dr_prev = 0;
            float cur_deg = ctrl_direct_target * (180.0f / 3.14159265f);
            if (joy_btn(BTN_DPAD_LEFT)  && !dl_prev) start_move_deg(cur_deg + 5.0f);
            if (joy_btn(BTN_DPAD_RIGHT) && !dr_prev) start_move_deg(cur_deg - 5.0f);
            dl_prev = joy_btn(BTN_DPAD_LEFT);
            dr_prev = joy_btn(BTN_DPAD_RIGHT);
          }
          break;
        }

        case STATE_SEQUENCE: {
          uint8_t  pairs         = (uint8_t)mb_slave.registers[0x22];
          uint32_t seq_dwell_t   = (uint32_t)(seq_dwell_ms  / 10.0f); if (seq_dwell_t  < 1) seq_dwell_t  = 1;
          uint32_t seq_timeout_t = (uint32_t)(seq_timeout_ms / 10.0f); if (seq_timeout_t < seq_dwell_t+1) seq_timeout_t = seq_dwell_t+1;

          #define SEQ_OK(reed) ((seq_mb_timer >= seq_dwell_t && (reed)) || seq_mb_timer >= seq_timeout_t)

          switch (seq_mb_state) {
            case SEQ_MB_IDLE:
              seq_mb_pair_idx = 0;
              seq_mb_step     = 0;
              seq_mb_state    = SEQ_MB_GOING_PICK;
              seq_mb_timer    = 0;
              break;

            case SEQ_MB_GOING_PICK: {
              mb_slave.registers[0x27] = 0x02;
              int16_t pick_raw = (int16_t)mb_slave.registers[0x12 + seq_mb_pair_idx * 2];
              if (seq_mb_timer == 0) {
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
                start_move_hole(pick_raw);
                seq_mb_timer = 1;
                break;
              }
              if (ctrl_settled) seq_mb_timer++;
              if (ctrl_settled && seq_mb_timer >= (uint32_t)(seq_settle_ms / 10.0f + 0.5f)) {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
                seq_mb_timer = 0; seq_mb_step = 0;
                seq_mb_state = SEQ_MB_PICKING;
              }
              break;
            }

            case SEQ_MB_PICKING:
              seq_mb_timer++;
              switch (seq_mb_step) {
                case 0:
                  if (SEQ_OK(reed_down)) {
                    HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);
                    seq_mb_timer = 0; seq_mb_step = 1;
                  }
                  break;
                case 1:
                  if (SEQ_OK(reed_grip)) {
                    HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
                    seq_mb_timer = 0; seq_mb_step = 2;
                  }
                  break;
                case 2:
                  if (SEQ_OK(reed_up)) {
                    seq_mb_timer = 0; seq_mb_step = 0;
                    seq_mb_state = SEQ_MB_GOING_PLACE;
                  }
                  break;
              }
              break;

            case SEQ_MB_GOING_PLACE: {
              mb_slave.registers[0x27] = 0x04;
              int16_t place_raw = (int16_t)mb_slave.registers[0x12 + seq_mb_pair_idx * 2 + 1];
              if (seq_mb_timer == 0) {
                start_move_hole(place_raw);
                seq_mb_timer = 1;
                break;
              }
              if (ctrl_settled) seq_mb_timer++;
              if (ctrl_settled && seq_mb_timer >= (uint32_t)(seq_settle_ms / 10.0f + 0.5f)) {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
                seq_mb_timer = 0; seq_mb_step = 0;
                seq_mb_state = SEQ_MB_PLACING;
              }
              break;
            }

            case SEQ_MB_PLACING:
              seq_mb_timer++;
              switch (seq_mb_step) {
                case 0:
                  if (SEQ_OK(reed_down)) {
                    HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
                    seq_mb_timer = 0; seq_mb_step = 1;
                  }
                  break;
                case 1:
                  if (SEQ_OK(!reed_grip)) {
                    HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
                    seq_mb_timer = 0; seq_mb_step = 2;
                  }
                  break;
                case 2:
                  if (SEQ_OK(reed_up)) {
                    seq_mb_pair_idx++;
                    seq_mb_timer = 0; seq_mb_step = 0;
                    if (seq_mb_pair_idx >= pairs) {
                      seq_mb_state = SEQ_MB_IDLE;
                      mb_slave.registers[0x22] = 0;
                      mb_slave.registers[0x27] = 0;
                      start_move_hole(0);
                      skip_p2p_entry = 1;
                      current_state  = STATE_AUTO;
                    } else {
                      seq_mb_state = SEQ_MB_GOING_PICK;
                    }
                  }
                  break;
              }
              break;

            default: seq_mb_state = SEQ_MB_IDLE; break;
          }
          #undef SEQ_OK

          if (mb_slave.registers[0x25] & 0x01) {
            control_reset();
            seq_mb_state = SEQ_MB_IDLE;
            seq_mb_step  = 0;
            HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GRIPPER_GPIO_Port,   GRIPPER_Pin,   GPIO_PIN_RESET);
            current_state = STATE_IDLE;
            mb_slave.registers[0x25] = 0;
            mb_slave.registers[0x27] = 0;
          }
          break;
        }

        case STATE_TEST: {
          uint8_t traj_done = ctrl_settled;
          float t_start = (float)(int16_t)mb_slave.registers[0x09];
          float t_end   = (float)(int16_t)mb_slave.registers[0x10];

          if (mb_slave.registers[0x06] & 0x01) {
            float tst_v = (float)(int16_t)mb_slave.registers[0x07] * (3.14159265f / 180.0f);
            float tst_a = (float)(int16_t)mb_slave.registers[0x08] * (3.14159265f / 180.0f);
            dev_dash.Traj.v_max        = tst_v;
            dev_dash.Traj.a_max        = tst_a;
            dev_dash.Ctrl.max_velocity = tst_v;
            dev_dash.Ctrl.max_accel    = tst_a;
          }

          mb_slave.registers[0x27] = 0x08;
          if (test_mv_armed) {
            start_move_deg((test_state == TEST_GOING_END) ? t_end : t_start);
            test_mv_armed = 0;
          }
          if (traj_done) {
            if (test_state == TEST_GOING_END) {
              test_state    = TEST_GOING_START;
              test_mv_armed = 1;
            } else {
              if (test_repeat > 0) test_repeat--;
              if (test_repeat == 0) {
                test_state    = TEST_IDLE;
                mb_slave.registers[0x27] = 0;
                current_state = STATE_IDLE;
              } else {
                test_state    = TEST_GOING_END;
                test_mv_armed = 1;
              }
            }
          }
          if (mb_slave.registers[0x25] & 0x01) {
            control_reset();
            test_state    = TEST_IDLE;
            test_mv_armed = 0;
            current_state = STATE_IDLE;
            mb_slave.registers[0x25] = 0;
            mb_slave.registers[0x27] = 0;
          }
          break;
        }

        case STATE_EMER:
          break;

        default:
          current_state = STATE_IDLE;
          break;
      } /* switch */

      /* ── Modbus Actuator Control ─────────────────────────────────────── */
      {
        if (seq_mb_state == SEQ_MB_IDLE && act_seq_state == ACT_IDLE) {
          static uint16_t prev_grip_cmd = 0xFFFF;
          uint16_t grip_cmd = mb_slave.registers[0x02];
          if (grip_cmd != prev_grip_cmd) {
            if (grip_cmd & 0x06) {
              if (grip_cmd & 0x02) HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
              if (grip_cmd & 0x04) HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);
            } else {
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin,
                                (grip_cmd & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            prev_grip_cmd = grip_cmd;
          }
        }

        {
          static uint16_t seq3_prev = 0;
          uint16_t seq3 = mb_slave.registers[0x03];
          if (act_seq_state == ACT_IDLE) {
            if ((seq3 & 0x01) && !(seq3_prev & 0x01)) {
              if (HAL_GPIO_ReadPin(GRIPPER_GPIO_Port, GRIPPER_Pin) != GPIO_PIN_RESET)
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
              act_seq_state = ACT_PK_DOWN;
              act_seq_timer = 0;
            } else if ((seq3 & 0x02) && !(seq3_prev & 0x02)) {
              HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
              act_seq_state = ACT_PL_DOWN;
              act_seq_timer = 0;
            }
          }
          if (seq3 != 0u) {
            mb_slave.registers[0x03] = 0u;
            seq3_prev = 0u;
          } else {
            seq3_prev = seq3;
          }

          act_seq_timeout = (uint32_t)(seq_dwell_ms / 10.0f);
          if (act_seq_timeout < 1) act_seq_timeout = 1;

          #define ACT_OK(reed) ((act_seq_timer >= act_seq_timeout && (reed)) || \
                                 act_seq_timer >= (uint32_t)(seq_timeout_ms / 10.0f))
          switch (act_seq_state) {
            case ACT_IDLE: break;
            case ACT_PK_DOWN:
              act_seq_timer++;
              if (ACT_OK(reed_down)) {
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);
                act_seq_state = ACT_PK_GRIP; act_seq_timer = 0;
              }
              break;
            case ACT_PK_GRIP:
              act_seq_timer++;
              if (ACT_OK(reed_grip)) {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
                act_seq_state = ACT_PK_UP; act_seq_timer = 0;
              }
              break;
            case ACT_PK_UP:
              act_seq_timer++;
              if (ACT_OK(reed_up)) {
                mb_slave.registers[0x27] |= 0x02;
                act_seq_state = ACT_IDLE;
              }
              break;
            case ACT_PL_DOWN:
              act_seq_timer++;
              if (ACT_OK(reed_down)) {
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
                act_seq_state = ACT_PL_OPEN; act_seq_timer = 0;
              }
              break;
            case ACT_PL_OPEN:
              act_seq_timer++;
              if (ACT_OK(!reed_grip)) {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
                act_seq_state = ACT_PL_UP; act_seq_timer = 0;
              }
              break;
            case ACT_PL_UP:
              act_seq_timer++;
              if (ACT_OK(reed_up)) {
                mb_slave.registers[0x27] |= 0x04;
                act_seq_state = ACT_IDLE;
              }
              break;
            #undef ACT_OK
            default: act_seq_state = ACT_IDLE; break;
          }
        }
      }

      /* ── Motor ramp apply (non-PID states) ───────────────────────────── */
      if (current_state != STATE_AUTO &&
          current_state != STATE_SEQUENCE &&
          current_state != STATE_TEST) {
        static float   prod_ramp     = 0.0f;
        static uint8_t prod_dead_cnt = 0;
        float prod_raw = motor_speed_cmd;
        if (current_state == STATE_EMER) prod_raw = 0.0f;
        if (soft_limit_dir > 0 && prod_raw > 0.0f) prod_raw = 0.0f;
        if (soft_limit_dir < 0 && prod_raw < 0.0f) prod_raw = 0.0f;
        float prod_target = (prod_raw >  dev_dash.Sys.max_speed) ?  dev_dash.Sys.max_speed :
                            (prod_raw < -dev_dash.Sys.max_speed) ? -dev_dash.Sys.max_speed : prod_raw;
        float prod_ramp_rate = dev_dash.Sys.ramp_rate;
        if ((prod_target > 0.0f && prod_ramp < 0.0f) ||
            (prod_target < 0.0f && prod_ramp > 0.0f)) {
          prod_dead_cnt = 5;
        }
        if (prod_dead_cnt > 0) { prod_ramp = 0.0f; prod_dead_cnt--; }
        else if (prod_target > prod_ramp + prod_ramp_rate) prod_ramp += prod_ramp_rate;
        else if (prod_target < prod_ramp - prod_ramp_rate) prod_ramp -= prod_ramp_rate;
        else                                                prod_ramp  = prod_target;
        safe_speed = prod_ramp;

        if (safe_speed >= 0.0f) {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
          safe_speed = -safe_speed;
        }
        if (safe_speed > 1.0f) safe_speed = 1.0f;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));
      }

    } /* end PRODUCTION MODE */
  } /* end !emer_active */

  /* ── Tower Lights + RESET_LED ────────────────────────────────────────── */
  if (dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {
    static uint8_t tl_tick = 0;
    tl_tick++;
    uint8_t G = 0, Y = 0, R = 0;

    if (emer_active) {
      uint8_t estop_pressed = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET);
      if (estop_pressed) {
        R = (tl_tick % 50) < 25 ? 1 : 0;
      } else {
        R = 1;
      }
      HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin,
        estop_pressed ? GPIO_PIN_RESET
                      : ((tl_tick % 50) < 25 ? GPIO_PIN_SET : GPIO_PIN_RESET));
    } else {
      HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_RESET);

      if (current_state == STATE_HOMING_FAST  ||
          current_state == STATE_HOMING_BACKOFF ||
          current_state == STATE_HOMING_SLOW) {
        Y = 0; G = 0; R = 0;
      } else if (!homed) {
        Y = (tl_tick % 50) < 25 ? 1 : 0;
        G = (tl_tick % 50) < 25 ? 0 : 1;
      } else if (current_state == STATE_IDLE   ||
                 current_state == STATE_MANUAL  ||
                 current_state == STATE_MANUAL_MB) {
        if (sethome_led_cnt > 0) {
          uint8_t c = sethome_led_cnt;
          Y = (c > 100) || (c > 60 && c <= 80) ? 1 : 0;
          G = (c <= 40) ? 1 : 0;
          if (++sethome_led_cnt > 130) sethome_led_cnt = 0;
        } else if (current_state == STATE_MANUAL || current_state == STATE_MANUAL_MB) {
          Y = 1;
        } else {
          G = 1;
        }
      } else if (current_state == STATE_AUTO) {
        float spd = fabsf(ctrl_vel_rad_s);
        if      (spd > 4.0f) G = (tl_tick % 10) < 5  ? 1 : 0;
        else if (spd > 1.0f) G = (tl_tick % 30) < 15 ? 1 : 0;
        else if (spd > 0.2f) G = (tl_tick % 60) < 30 ? 1 : 0;
        else                 G = 1;
      } else if (current_state == STATE_SEQUENCE) {
        G = (tl_tick % 50) < 25 ? 1 : 0;
        Y = 1;
      } else if (current_state == STATE_TEST) {
        G = 1;
        Y = (tl_tick % 50) < 25 ? 1 : 0;
      }

      if (soft_limit_dir != 0) { R = 1; G = 1; Y = 0; }
    }

    HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, G ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, Y ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, R ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  /* ── Live Expressions dashboard update ───────────────────────────────── */
  dev_dash.IO.joy_connected = joy_is_connected();
  dev_dash.IO.joy_buttons   = joy_raw_buttons();
  dev_dash.IO.joy_ly        = joy_ly_f();
  dev_dash.IO.joy_rt        = joy_rt_f();
  dev_dash.IO.joy_lt        = joy_lt_f();

  dev_dash.Status.status_state = current_state;
  dev_dash.Status.motor_cmd    = (current_state == STATE_AUTO)
                                 ? motor_speed_cmd
                                 : ((HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_SET)
                                    ? safe_speed : -safe_speed);
  dev_dash.Status.encoder_raw = current_position;
  dev_dash.Status.current_A   = current_sensor_A;
  dev_dash.Status.pos_rad     = current_position * RAD_PER_CNT;
  {
    int32_t pos_mod = (int32_t)current_position % 8192;
    if (pos_mod < 0) pos_mod += 8192;
    dev_dash.Status.pos_deg = (float)pos_mod * (360.0f / 8192.0f);
  }
  dev_dash.Status.vel_rad_s  = ctrl_vel_rad_s;
  dev_dash.Status.acc_rad_s2 = ctrl_acc_rad_s2;

  dev_dash.IO.in_estop  = HAL_GPIO_ReadPin(ESTOP_GPIO_Port,     ESTOP_Pin);
  dev_dash.IO.in_mode   = HAL_GPIO_ReadPin(MODE_GPIO_Port,      MODE_Pin);
  dev_dash.IO.in_reset  = HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin);
  dev_dash.IO.in_power  = HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin);

  dev_dash.IO.out_pwm         = (safe_speed > 0.0f || safe_speed < 0.0f) ? 1 : 0;
  dev_dash.IO.out_dir         = HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port,    MOTOR_DIR_Pin);
  dev_dash.IO.out_power_latch = HAL_GPIO_ReadPin(POWER_LATCH_GPIO_Port,  POWER_LATCH_Pin);
  dev_dash.IO.out_pneumatic   = HAL_GPIO_ReadPin(PNEUMATIC_GPIO_Port,    PNEUMATIC_Pin);
  dev_dash.IO.out_gripper     = HAL_GPIO_ReadPin(GRIPPER_GPIO_Port,      GRIPPER_Pin);
  dev_dash.IO.reed_up         = reed_up;
  dev_dash.IO.reed_down       = reed_down;
  dev_dash.IO.reed_grip       = reed_grip;
  dev_dash.IO.out_tower_g     = HAL_GPIO_ReadPin(TOWER_G_GPIO_Port,      TOWER_G_Pin);
  dev_dash.IO.out_tower_y     = HAL_GPIO_ReadPin(TOWER_Y_GPIO_Port,      TOWER_Y_Pin);
  dev_dash.IO.out_tower_r     = HAL_GPIO_ReadPin(TOWER_R_GPIO_Port,      TOWER_R_Pin);
  dev_dash.IO.out_reset_led   = HAL_GPIO_ReadPin(RESET_LED_GPIO_Port,    RESET_LED_Pin);
  dev_dash.IO.out_emer        = HAL_GPIO_ReadPin(EMER_OUTPUT_GPIO_Port,  EMER_OUTPUT_Pin);

  /* ── Telemetry @ 50Hz ────────────────────────────────────────────────── */
  static uint8_t telem_div = 0;
  if (enable_telemetry && ++telem_div >= 2) { telem_div = 0; Send_Telemetry(); }

  /* ── IWDG refresh ────────────────────────────────────────────────────── */
  HAL_IWDG_Refresh(&hiwdg);

  /* ── Heartbeat LED ───────────────────────────────────────────────────── */
  static uint32_t last_hb = 0;
  if (++last_hb > 50) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    last_hb = 0;
  }
}
