#include "modbus_app.h"
#include "modbus.h"
#include "pid_control.h"
#include <math.h>

extern ModbusSlave_t mb_slave;

void modbus_app_receive(void)
{
  /* ── Python GUI / step command (reg 10–15, 0x38–0x3F) ───────────────── */
  {
    static uint8_t python_step_active = 0;
    int16_t step_cmd = (int16_t)mb_slave.registers[MB_REG_PYTHON_STEP];
    if (step_cmd != 0) {
      dev_dash.Sys.mode = SYS_MODE_HARDWARE_TEST;
      dev_dash.Test.force_motor = (float)step_cmd / 10000.0f;
      python_step_active = 1;
    } else if (python_step_active) {
      dev_dash.Sys.mode = SYS_MODE_PRODUCTION;
      dev_dash.Test.force_motor = 0.0f;
      python_step_active = 0;
    }

    if (mb_slave.registers[MB_REG_PYTHON_APPLY] == 1) {
      kp_vel = (float)mb_slave.registers[MB_REG_PYTHON_KP] / 100.0f;
      ki_vel = (float)mb_slave.registers[MB_REG_PYTHON_KI] / 100.0f;
      kd_vel = (float)mb_slave.registers[MB_REG_PYTHON_KD] / 100.0f;

      float target_rad = (float)((int16_t)mb_slave.registers[MB_REG_PYTHON_POS]) / 1000.0f;
      dev_dash.Cmd.target_deg = target_rad * (180.0f / 3.14159265f);

      if (mb_slave.registers[MB_REG_TRAJ_TYPE] <= 2)
        dev_dash.Traj.traj_type = (uint8_t)mb_slave.registers[MB_REG_TRAJ_TYPE];
      if (mb_slave.registers[MB_REG_VMAX] > 0)
        dev_dash.Traj.v_max = (float)mb_slave.registers[MB_REG_VMAX] / 100.0f;
      if (mb_slave.registers[MB_REG_AMAX] > 0)
        dev_dash.Traj.a_max = (float)mb_slave.registers[MB_REG_AMAX] / 100.0f;
      if (mb_slave.registers[MB_REG_JMAX] > 0)
        dev_dash.Traj.j_max = (float)mb_slave.registers[MB_REG_JMAX] / 100.0f;
      if (mb_slave.registers[MB_REG_KP_POS] > 0)
        kp_pos = (float)mb_slave.registers[MB_REG_KP_POS] / 100.0f;
      kd_pos = (float)mb_slave.registers[MB_REG_KD_POS] / 100.0f;
      ki_pos = (float)mb_slave.registers[MB_REG_KI_POS] / 100.0f;

      dev_dash.Cmd.start_move = 1;
      if (current_state != STATE_EMER) {
        skip_p2p_entry = 1;
        current_state = STATE_AUTO;
      }
      mb_slave.registers[MB_REG_PYTHON_APPLY] = 0;
    }
  }

  /* ── Set Home command (reg[21]) ──────────────────────────────────────── */
  if (mb_slave.registers[MB_REG_SET_HOME] == 1) {
    dev_dash.Cmd.set_home = 1;
    mb_slave.registers[MB_REG_SET_HOME] = 0;
  }

  /* ── Soft Stop (reg[25]) ─────────────────────────────────────────────── */
  if (mb_slave.registers[MB_REG_STOP] == 1) {
    mb_slave.registers[MB_REG_STOP] = 0;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    motor_speed_cmd    = 0.0f;
    vel_filtered       = 0.0f;
    float hold = my_encoder.position_rad;
    ctrl_direct_target = hold;
    ctrl_traj_start    = hold;
    dev_dash.Cmd.target_deg = hold * (180.0f / 3.14159f);
    dev_dash.Cmd.start_move = 0;
    control_reset();
  }

  /* ── dev_dash.Ctrl sync ──────────────────────────────────────────────── */
  {
    DashCtrl_t *C = &dev_dash.Ctrl;

    control_model[0].traj_type = C->traj_type;
    control_model[0].v_max     = C->max_velocity;
    control_model[0].a_max     = C->max_accel;
    control_model[0].j_max     = C->max_jerk;
    control_model[0].kp_pos    = C->kp_position;
    control_model[0].ki_pos    = C->ki_pos;
    control_model[0].kd_pos    = C->kd_pos;
    control_model[0].kp_vel    = C->kp_vel;
    control_model[0].ki_vel    = C->ki_vel;
    control_model[0].kd_vel    = C->kd_vel;
    control_model[1]           = control_model[0];
    ctrl_pos_deadband_deg      = C->pos_deadband_deg;
    ctrl_vel_deadband          = C->vel_deadband;
    sys_ff_enabled             = C->refff_enabled;
    sys_V_supply               = C->V_supply;
    refff_enabled              = C->refff_enabled;
    V_supply                   = C->V_supply;
    motor_dir_inverted         = C->motor_dir_inverted;
    kp_vel = C->kp_vel;  ki_vel = C->ki_vel;  kd_vel = C->kd_vel;
    kp_pos = C->kp_position; ki_pos = C->ki_pos; kd_pos = C->kd_pos;

    if (C->start_move) {
      C->start_move = 0;
      dev_dash.Cmd.target_deg = C->traj_target_deg;
      dev_dash.Cmd.start_move = 1;
      if (current_state != STATE_EMER) { skip_p2p_entry = 1; current_state = STATE_AUTO; }
    }
    if (C->reset_all) {
      C->reset_all = 0;
      control_reset();
      motor_speed_cmd = 0.0f;
    }
    if (C->zero_encoder) {
      C->zero_encoder = 0;
      dev_dash.Cmd.set_home = 1;
    }

    /* R/O observables → Ctrl */
    C->ss_error_rad     = ctrl_pos_err;
    C->ss_error_deg     = ctrl_pos_err * (180.0f / 3.14159265f);
    C->ss_reached       = ctrl_settled;
    C->vel_error_live   = ctrl_vel_err;
    C->i_term_live      = vel_integral;
    C->g_traj_pos       = ctrl_ideal_pos * (180.0f / 3.14159265f);
    C->g_smooth_vel     = vel_filtered * (180.0f / 3.14159265f);
    C->g_smooth_accel   = ctrl_acc_rad_s2 * (180.0f / 3.14159265f);
    C->g_position_rad   = my_encoder.position_rad;
    C->g_velocity_rad_s = vel_filtered;
    C->g_pwm_duty       = motor_speed_cmd;
    C->g_motor_voltage  = motor_speed_cmd * C->V_supply;
    C->g_current_A      = current_sensor_A;
    C->pos_integral_live = pos_integral;
    C->seq_step_delay   = seq_dwell_ms / 1000.0f;
  }

  /* ── Heartbeat (reg[0x00] YA/HI) ────────────────────────────────────── */
  {
    static uint32_t hb_timer = 0;
    hb_timer++;
    if (mb_slave.registers[MB_REG_HEARTBEAT] == 18537) {
      hb_timer = 0;
      mb_slave.registers[MB_REG_HEARTBEAT] = 22881;
    } else if (hb_timer > 300) {
      /* 3-second timeout — UI disconnected */
    }
  }

  /* ── Hardware selector switch ────────────────────────────────────────── */
  {
    static GPIO_PinState last_mode_switch = (GPIO_PinState)2;
    static uint8_t mode_switch_initialized = 0;
    GPIO_PinState current_mode_switch = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
    if (current_mode_switch != last_mode_switch) {
      if (!mode_switch_initialized) {
        mode_switch_initialized = 1;
      } else if (current_state != STATE_EMER && current_state != STATE_SEQUENCE &&
                 current_state != STATE_TEST && dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {
        if (current_mode_switch == GPIO_PIN_SET) {
          motor_speed_cmd = 0.0f;
          control_reset();
          current_state = STATE_MANUAL;
        } else {
          motor_speed_cmd = 0.0f;
          current_state = STATE_IDLE;
        }
      }
      last_mode_switch = current_mode_switch;
    }
  }
}

void modbus_app_send(void)
{
  mb_slave.registers[MB_REG_STATE] = (uint16_t)current_state;
  mb_slave.registers[MB_REG_HOMED] = (uint16_t)homed;
  mb_slave.registers[MB_REG_REED]  = (reed_up   ? 1 : 0) |
                                     (reed_down ? 2 : 0) |
                                     (reed_grip ? 4 : 0);

  /* Position: relative to SET HOME point, wrapped 0-359.99° × 10 */
  {
    float pd = cumulative_angle_deg - home_offset_deg;
    pd = fmodf(pd, 360.0f);
    if (pd < 0.0f) pd += 360.0f;
    mb_slave.registers[MB_REG_POS] = (int16_t)(pd * 10.0f);
  }

  mb_slave.registers[MB_REG_VEL]    = (int16_t)(vel_filtered * (180.0f / 3.14159265f) * 10.0f);
  mb_slave.registers[MB_REG_ACC]    = (int16_t)(ctrl_acc_rad_s2 * (180.0f / 3.14159265f) * 10.0f);
  mb_slave.registers[MB_REG_ESTOP]  = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
  mb_slave.registers[MB_REG_CURRENT_MA] = (uint16_t)(current_sensor_A * 1000.0f);

  mb_slave.registers[MB_REG_DIG_IO] =
      (home_sensor_raw ? 0x01 : 0) |
      ((HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port,  POWER_BTN_Pin)  == GPIO_PIN_RESET) ? 0x02 : 0) |
      ((HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port,  RESET_BTN_Pin)  == GPIO_PIN_RESET) ? 0x04 : 0) |
      ((HAL_GPIO_ReadPin(MODE_GPIO_Port,        MODE_Pin)       == GPIO_PIN_SET)   ? 0x08 : 0);

  if (mb_slave.registers[MB_REG_SYS_MODE] <= 3)
    dev_dash.Sys.mode = (SystemMode_t)mb_slave.registers[MB_REG_SYS_MODE];
  else
    mb_slave.registers[MB_REG_SYS_MODE] = (uint16_t)dev_dash.Sys.mode;

  if (mb_slave.registers[MB_REG_MAX_SPEED] >= 1 && mb_slave.registers[MB_REG_MAX_SPEED] <= 100)
    dev_dash.Sys.max_speed = (float)mb_slave.registers[MB_REG_MAX_SPEED] / 100.0f;
  else
    mb_slave.registers[MB_REG_MAX_SPEED] = (uint16_t)(dev_dash.Sys.max_speed * 100.0f);

  if (mb_slave.registers[MB_REG_RAMP_RATE] >= 1 && mb_slave.registers[MB_REG_RAMP_RATE] <= 500)
    dev_dash.Sys.ramp_rate = (float)mb_slave.registers[MB_REG_RAMP_RATE] / 1000.0f;
  else
    mb_slave.registers[MB_REG_RAMP_RATE] = (uint16_t)(dev_dash.Sys.ramp_rate * 1000.0f);

  encoder_inverted = (uint8_t)(mb_slave.registers[MB_REG_ENC_INV] & 0x01);
}
