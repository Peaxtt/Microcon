/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "robot.h"
#include "hardware.h"
#include "modbus_app.h"
#include "state_machine.h"
#include "joystick.h"
#include "modbus.h"
#include "pid_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
uint32_t adc_dma_buf[40];          /* ADC2 circular DMA buffer (PA0 current sensor) */
uint8_t  joy_dma_buf[PKT_LEN * 2];
ModbusSlave_t mb_slave;
uint8_t  skip_p2p_entry = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
void Send_Telemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE { return ch; }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    for (uint16_t i = 0; i + PKT_LEN <= Size; i++) {
      if (joy_dma_buf[i] == 0xAA) {
        joystick_parse(&joy_dma_buf[i]);
        break;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, joy_dma_buf, sizeof(joy_dma_buf));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
  if (huart->Instance == LPUART1)
  {
    modbus_rx_cplt(&mb_slave, Size);
  }
}

// Re-arm USART3 DMA after any UART error (framing, overrun, noise)
// Without this, DMA stops silently and joystick reads freeze at 0
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, joy_dma_buf, sizeof(joy_dma_buf));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* ── PCB I/O TEST ───────────────────────────────────────────────────────
   * Validates new pin assignment before IOC commit.
   * Open terminal 115200 8N1. Function blocks at the end; reflash after.
   * To switch tests: comment one line, uncomment the other.
   * ----------------------------------------------------------------------- */
  // pcb_io_test(&hlpuart1);           /* sequential terminal test */
  // pin_short_test(&hlpuart1);

  //pcb_interactive_test(&hlpuart1);    /* web UI — เปิด docs/pin_tester.html */

  // EMER (ESTOP_Pin PB13): active LOW, PULLUP — polled in 100Hz loop
  // MODE selector: PULLUP (LOW=AUTO, HIGH=MANUAL)
  {
    GPIO_InitTypeDef g = {0};
    g.Pin  = ESTOP_Pin;
    g.Mode = GPIO_MODE_INPUT;   // poll-based, no EXTI needed
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ESTOP_GPIO_Port, &g);
    g.Pin  = MODE_Pin;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MODE_GPIO_Port, &g);
  }

  joystick_init(&huart3);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, joy_dma_buf, sizeof(joy_dma_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

  modbus_init(&mb_slave, &hlpuart1);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  Encoder_Init(&my_encoder, LOOP_DT);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  
  // Start the 1kHz Control Loop
  HAL_TIM_Base_Start_IT(&htim7);
  
  // Latch Power On (Commented out as PB14 is now TOWER_R)
  // HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);
  
  // Refresh Watchdog
  HAL_IWDG_Refresh(&hiwdg);
  
  // HOME_SENSOR: plain input (EXTI enabled only during homing)
  homing_exti_disable();
  cumulative_angle_deg = 0.0f; /* TODO: load from Flash when ready */
  homed = 0;

  current_state = STATE_IDLE;
  mb_slave.registers[0x00] = 22881; // Initialize Heartbeat (YA)
  // Sys params initial values — readable+writable by Modbus
  mb_slave.registers[0x33] = (uint16_t)dev_dash.Sys.mode;
  mb_slave.registers[0x34] = (uint16_t)(dev_dash.Sys.max_speed  * 100.0f);
  mb_slave.registers[0x35] = (uint16_t)(dev_dash.Sys.ramp_rate  * 1000.0f);
  mb_slave.registers[0x36] = (uint16_t)encoder_inverted;
  
  // Calibrate + start ADC2 DMA (circular, feeds adc_dma_buf continuously)
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, adc_dma_buf, 40);
  HAL_Delay(100); // let buffer fill (~2.4ms/cycle, 100ms >> enough)
  // Auto-zero: average current DMA buffer as zero-current offset
  {
    uint32_t zsum = 0;
    for (int i = 0; i < 40; i++) zsum += adc_dma_buf[i];
    dev_dash.Sys.cur_zero_v = ((float)(zsum / 40u) / 4095.0f) * 3.3f;
  }

  printf("\r\n=== 1-DOF Robot System Ready ===\r\n");

  // Init trajectory generators (kept for python_gui telemetry / dev_dash.Traj reference)
  Trapezoid_Init(&g_trapezoid, dev_dash.Traj.v_max, dev_dash.Traj.a_max, 0.001f);
  SCurve_Init(&g_scurve, dev_dash.Traj.v_max, dev_dash.Traj.a_max, dev_dash.Traj.j_max, 0.001f);

  // Init control.c — initialises internal SCurve/Trapezoid/RefFF/DistFF
  control_init();

  // Legacy RefFF/DistFF for python_gui telemetry (control.c has its own instances)
  RefFF_Init(&my_refff,
      0.027f,      // J_eq  kg·m²
      0.0012794f,  // L     H
      2.8f,        // R     Ω
      0.3f,        // b_eq  N·m·s/rad
      50.0f,       // N     gear ratio
      0.0045f,     // Kt    N·m/A
      0.005f,      // Ke    V·s/rad
      0.02f,       // tau   s
      0.001f);     // Ts    s (1kHz)

  DistFF_Init(&my_distff,
      0.0012794f,  // L
      2.8f,        // R
      50.0f,       // N
      0.0045f,     // Kt
      0.02f,       // tau
      0.001f);     // Ts

  /* (pin_short_test call moved to top of USER CODE BEGIN 2) */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    modbus_process(&mb_slave); // Run every loop — respond immediately, not gated by 10ms flag

    if (flag_10ms) {
      flag_10ms = 0;

      hardware_sensors_update();
      modbus_process(&mb_slave);
      modbus_app_receive();
      modbus_app_send();

      /* ESTOP debounce — same position as original main.c (before state machine) */
      {
        static uint8_t estop_debounce = 0;
        if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) {
          if (estop_debounce < 25) estop_debounce++;
          if (estop_debounce >= 20) {
            current_state   = STATE_EMER;
            motor_speed_cmd = 0.0f;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
          }
        } else {
          estop_debounce = 0;
        }
      }

      state_machine_tick();
    }
  }
  /* USER CODE END 3 */
}

/* ── DEAD CODE REMOVED — kept below as separator ── */
#if 0
      // ── EMER: ทำงานทุกโหมด (HW Test, Joy Test, Auto Test, Production) ─────
      float safe_speed = 0.0f; // stays 0 when EMER active
      uint8_t emer_active = (current_state == STATE_EMER);
      if (emer_active) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
        mb_slave.registers[0x27] = 0;
        motor_speed_cmd = 0.0f;
        // ออก EMER: ต้องปล่อย ESTOP **และ** กด RESET ค้างไว้ 50ms (5 ticks × 10ms)
        static uint8_t emer_rel_cnt = 0;
        uint8_t estop_released = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port,   ESTOP_Pin)     == GPIO_PIN_SET);
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
          float hold_pos = current_position * RAD_PER_CNT;
          ctrl_direct_target       = hold_pos;
          ctrl_traj_start          = hold_pos;
          dev_dash.Cmd.target_deg  = hold_pos * (180.0f / 3.14159f);
          mb_slave.registers[0x24] = 0;
          mb_slave.registers[0x01] = 0;
          mb_slave.registers[0x05] = 0;  /* clear pending jog — prevents auto-restart */
          mb_slave.registers[0x25] = 0;
          control_reset();
          motor_speed_cmd = 0.0f;
          /* homed stays as-is after EMER exit — user decides when to re-home */
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);
          /* TOWER_R and RESET_LED cleared by tower lights on next tick */
          if (dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {
            GPIO_PinState mode_now = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
            current_state = (mode_now == GPIO_PIN_RESET) ? STATE_IDLE : STATE_MANUAL;
          } else {
            current_state = STATE_IDLE;
          }
        }
      }

      // ── SET HOME: RT + A (any non-EMER state) ────────────────────────────────
      if (!emer_active) {
        if (joy_is_connected() && joy_rt_f() > 0.5f && joy_btn(BTN_A)) {
          dev_dash.Cmd.set_home = 1;
        }
        if (dev_dash.Cmd.set_home) {
          dev_dash.Cmd.set_home = 0;
          /* 1. Stop motor immediately */
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
          motor_speed_cmd = 0.0f;
          /* 2. Store sensor→home distance */
          home_offset_deg = cumulative_angle_deg;
          /* 3. Re-zero encoder at this point */
          __disable_irq();
          enc_reset_pending    = 1;
          cumulative_angle_deg = 0.0f;
          ctrl_direct_target   = 0.0f;
          ctrl_traj_start      = 0.0f;
          __enable_irq();
          /* 4. Reset PID with target = current pos (0) — no movement */
          control_reset();
          control_set_target(0.0f);
          homed           = 1;
          sethome_led_cnt = 1;
        }
      }

      // 4. System Mode Selector — ข้ามเมื่อ EMER active
      if (!emer_active) {

      if (dev_dash.Sys.mode == SYS_MODE_HARDWARE_TEST) {
        
        // --- HARDWARE TEST UI MODE ---
        // Bypass State Machine and Modbus commands completely.
        // Apply physical outputs directly from the Live Expressions variables.
        
        HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, dev_dash.Test.force_pneumatic ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, dev_dash.Test.force_gripper ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, dev_dash.Test.force_tower_g ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, dev_dash.Test.force_tower_y ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, dev_dash.Test.force_tower_r ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, dev_dash.Test.force_emer ? GPIO_PIN_RESET : GPIO_PIN_SET);
        
        // Apply Forced Motor Speed
        safe_speed = dev_dash.Test.force_motor;
        if (safe_speed > 1.0f) safe_speed = 1.0f;
        if (safe_speed < -1.0f) safe_speed = -1.0f;
        
        if (safe_speed >= 0) {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
          safe_speed = -safe_speed;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else if (dev_dash.Sys.mode == SYS_MODE_JOYSTICK_TEST) {

        // --- JOYSTICK HARDWARE TEST MODE ---
        // Bypass State Machine. Map joystick buttons directly to physical pins for fast testing.
        // LB = Pick sequence, RB = Place sequence (non-blocking, configurable seq_delay_s)

        typedef enum {
          SEQ_IDLE = 0,
          SEQ_PICK_OPEN_WAIT, // gripper opened, wait before going down
          SEQ_PICK_WAIT1,     // down (POWER_LATCH ON), wait before grip
          SEQ_PICK_WAIT2,     // gripped, wait before going up
          SEQ_PLACE_WAIT1,    // down (POWER_LATCH ON), wait before release
          SEQ_PLACE_WAIT2,    // released, wait before going up
        } SeqState_t;
        static SeqState_t seq_state   = SEQ_IDLE;
        static uint32_t   seq_timer   = 0;     // counts 100Hz ticks
        static uint8_t    lb_prev     = 0;
        static uint8_t    rb_prev     = 0;
        static uint8_t    y_prev      = 0;
        static uint8_t    gripper_latch = 0;   // toggle state for gripper (NC/NO)

        if (joy_is_connected()) {

          // --- Sequence state machine (100 Hz) ---
          uint32_t seq_ticks = (uint32_t)(seq_delay_s * 100.0f);
          if (seq_ticks < 1) seq_ticks = 1;

          switch (seq_state) {

            case SEQ_IDLE:
              if (joy_btn(BTN_LB) && !lb_prev) {
                // PICK: เปิด gripper ก่อนเสมอ แล้วค่อยลง
                gripper_latch = 0;
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
                seq_timer = 0;
                seq_state = SEQ_PICK_OPEN_WAIT;
              } else if (joy_btn(BTN_RB) && !rb_prev) {
                // PLACE: ลงก่อน
                HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);
                seq_timer = 0;
                seq_state = SEQ_PLACE_WAIT1;
              }
              break;

            /* ---- PICK: เปิด gripper → รอ → ลง → รอ → หนีบ → รอ → ขึ้น ---- */
            case SEQ_PICK_OPEN_WAIT:
              if (++seq_timer >= seq_ticks) {
                HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET); // ลง
                seq_timer = 0;
                seq_state = SEQ_PICK_WAIT1;
              }
              break;
            case SEQ_PICK_WAIT1:
              if (++seq_timer >= seq_ticks) {
                gripper_latch = 1;
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET); // หนีบ
                seq_timer = 0;
                seq_state = SEQ_PICK_WAIT2;
              }
              break;
            case SEQ_PICK_WAIT2:
              if (++seq_timer >= seq_ticks) {
                HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET); // ขึ้น
                seq_state = SEQ_IDLE;
              }
              break;

            /* ---- PLACE: ลง → รอ → ปล่อย → รอ → ขึ้น ---- */
            case SEQ_PLACE_WAIT1:
              if (++seq_timer >= seq_ticks) {
                gripper_latch = 0;
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET); // ปล่อย
                seq_timer = 0;
                seq_state = SEQ_PLACE_WAIT2;
              }
              break;
            case SEQ_PLACE_WAIT2:
              if (++seq_timer >= seq_ticks) {
                HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET); // ขึ้น
                seq_state = SEQ_IDLE;
              }
              break;

            default:
              seq_state = SEQ_IDLE;
              break;
          }

          lb_prev = joy_btn(BTN_LB);
          rb_prev = joy_btn(BTN_RB);

          // --- Direct actuator control (only when no sequence running) ---
          // B = ลง (POWER_LATCH) hold,  Y = หนีบ toggle (NC/NO latch),  X = PNEUMATIC hold
          if (seq_state == SEQ_IDLE) {
            HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, joy_btn(BTN_B) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,   PNEUMATIC_Pin,   joy_btn(BTN_X) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            // Y = toggle gripper latch
            if (joy_btn(BTN_Y) && !y_prev) {
              gripper_latch ^= 1;
              HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, gripper_latch ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
          }
          y_prev = joy_btn(BTN_Y);

          // Face button A = EMER OUTPUT (press A = trigger emergency = LOW)
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, joy_btn(BTN_A) ? GPIO_PIN_RESET : GPIO_PIN_SET);

          // DPAD Up/Down = Lights,  Left/Right = fine tune motor
          HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, joy_btn(BTN_DPAD_UP)   ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, joy_btn(BTN_DPAD_DOWN) ? GPIO_PIN_SET : GPIO_PIN_RESET);

          uint8_t fine_left  = joy_btn(BTN_DPAD_LEFT);
          uint8_t fine_right = joy_btn(BTN_DPAD_RIGHT);

          static float joy_ramp = 0.0f;
          static uint8_t joy_dead_cnt = 0;

          if (fine_left || fine_right) {
            // Fine tune: bypass ramp, apply minimum configurable speed directly
            float ft = homing_slow_speed;
            if (ft < 0.01f) ft = 0.01f;
            if (ft > 1.0f)  ft = 1.0f;
            safe_speed = fine_left ? ft : -ft;
            joy_ramp = 0.0f;  // reset so stick doesn't jump after fine tune
          } else {
            // Normal: RT + Left-stick Y with ramp
            float joy_raw = (joy_rt_f() > 0.5f) ? joy_ly_f() : 0.0f;
            float joy_target = (joy_raw >  dev_dash.Sys.max_speed) ?  dev_dash.Sys.max_speed :
                               (joy_raw < -dev_dash.Sys.max_speed) ? -dev_dash.Sys.max_speed : joy_raw;
            float ramp_rate_local = dev_dash.Sys.ramp_rate;
            if ((joy_target > 0.0f && joy_ramp < 0.0f) ||
                (joy_target < 0.0f && joy_ramp > 0.0f)) {
              joy_dead_cnt = 5;
            }
            if (joy_dead_cnt > 0) { joy_ramp = 0.0f; joy_dead_cnt--; }
            else if (joy_target > joy_ramp + ramp_rate_local) joy_ramp += ramp_rate_local;
            else if (joy_target < joy_ramp - ramp_rate_local) joy_ramp -= ramp_rate_local;
            else                                               joy_ramp  = joy_target;
            safe_speed = joy_ramp;
          }

          if (safe_speed >= 0) {
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
          } else {
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            safe_speed = -safe_speed;
          }
          if (safe_speed > 1.0f) safe_speed = 1.0f;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

        } else {
          // Safe state if joystick disconnects during test
          seq_state = SEQ_IDLE;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
          HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GRIPPER_GPIO_Port,     GRIPPER_Pin,     GPIO_PIN_RESET);
        }

      } else if (dev_dash.Sys.mode == SYS_MODE_AUTO_MOTOR_TEST) {

        // --- AUTO MOTOR TEST MODE ---
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
          auto_test_dir = !auto_test_dir;
          auto_dead_cnt = 5; // dead-time 50ms ก่อนสลับทิศ
        }
        float auto_target = dev_dash.Test.test_speed;
        if (auto_target > dev_dash.Sys.max_speed) auto_target = dev_dash.Sys.max_speed;
        if (auto_target < 0.0f) auto_target = 0.0f;

        if (auto_dead_cnt > 0) { auto_ramp = 0.0f; auto_dead_cnt--; }
        else if (auto_target > auto_ramp + dev_dash.Sys.ramp_rate) auto_ramp += dev_dash.Sys.ramp_rate;
        else if (auto_target < auto_ramp - dev_dash.Sys.ramp_rate) auto_ramp -= dev_dash.Sys.ramp_rate;
        else                                                          auto_ramp  = auto_target;

        safe_speed = auto_ramp;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                          auto_test_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else {

        // --- NORMAL PRODUCTION MODE ---
        // ตรวจจับ entry เข้า PID states → init target = ตำแหน่งปัจจุบัน ป้องกันหมุนทันที
        static RobotState_t prev_state = STATE_INIT;
        uint8_t entering_pid_state = (
          (current_state == STATE_AUTO     && prev_state != STATE_AUTO)     ||
          (current_state == STATE_SEQUENCE && prev_state != STATE_SEQUENCE) ||
          (current_state == STATE_TEST     && prev_state != STATE_TEST)
        );
        if (entering_pid_state && !homing_final_zero_pending) {
          float pos_now = current_position * RAD_PER_CNT;
          ctrl_direct_target = pos_now;
          ctrl_traj_start    = pos_now;
          if (!dev_dash.Cmd.start_move)
            dev_dash.Cmd.target_deg = pos_now * (180.0f / 3.14159f);
          control_reset();
          control_set_target(pos_now);
          mb_slave.registers[0x05] = 0;
        }
        prev_state = current_state;

        /* ── Post-homing proximity offset: re-zero encoder once arm settles ──────
           finish_homing() sets homing_final_zero_pending=1 and moves to offset pos.
           When settled here, zero the encoder so hole-0 = 0° exactly.            */
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
            home_offset_deg = 0.0f; /* arrived at home, reset offset */
          }
          current_state = STATE_IDLE;
        }

        // ── LT+LB: HOME via proximity sensor (global) ────────────────────────────
        if (joy_is_connected() && !emer_active) {
          static uint8_t ltlb_prev_g = 0;
          uint8_t ltlb_g = (joy_lt_f() > 0.5f && joy_btn(BTN_LB));
          if (ltlb_g && !ltlb_prev_g &&
              current_state != STATE_HOMING_FAST &&
              current_state != STATE_HOMING_BACKOFF &&
              current_state != STATE_HOMING_SLOW) {
            motor_speed_cmd          = 0.0f;
            control_reset();
            homing_exti_enable();
            current_state = STATE_HOMING_FAST;
          }
          ltlb_prev_g = ltlb_g;
        }

        // ── GO HOME: LT + A → กลับ SET HOME position ────────────────────────────
        if (joy_is_connected() && !emer_active) {
          static uint8_t lta_prev_g = 0;
          uint8_t lta_g = (joy_lt_f() > 0.5f && joy_btn(BTN_A));
          if (lta_g && !lta_prev_g && homed &&
              (current_state == STATE_IDLE || current_state == STATE_AUTO ||
               current_state == STATE_MANUAL || current_state == STATE_MANUAL_MB)) {
            /* GO HOME: drive to 0° (SET HOME point), no re-zero on arrival */
            homing_final_zero_pending = 0;
            homing_rezero_on_arrive   = 0;
            skip_p2p_entry = 1;
            current_state  = STATE_AUTO;
            start_move_deg(0.0f);
          }
          lta_prev_g = lta_g;
        }

        // ── Go Home: LT + B → วิ่งกลับ 0° ทางที่ใกล้ที่สุดด้วย PID ───────────────
        if (joy_is_connected() && joy_lt_f() > 0.5f && joy_btn(BTN_B)) {
          if (current_state == STATE_IDLE    || current_state == STATE_MANUAL ||
              current_state == STATE_MANUAL_MB || current_state == STATE_AUTO) {
            dev_dash.Cmd.target_deg = 0.0f;
            dev_dash.Cmd.start_move = 1;
            skip_p2p_entry = 1;
            current_state = STATE_AUTO;
          }
        }

        // ── Global Commands (ทำงานทุก state) ─────────────────────────────────

        // reg[0x05] jog — cumulative from current pos; BaseSystem +N=CCW encoder +CW → negate
        {
          int16_t jog_g = (int16_t)mb_slave.registers[0x05];
          if (entering_pid_state) mb_slave.registers[0x05] = 0; /* clear stale jog on state entry */
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

        // reg[0x01] mode commands
        if (current_state != STATE_EMER &&
            current_state != STATE_HOMING_FAST &&
            current_state != STATE_HOMING_BACKOFF &&
            current_state != STATE_HOMING_SLOW) {
          uint16_t mode_cmd = mb_slave.registers[0x01];
          if (mode_cmd & 0x01) {  // HOME
            if (homed) {
              /* Already homed: go to 0° (SET HOME point) same as LT+A */
              homing_final_zero_pending = 0;
              homing_rezero_on_arrive   = 0;
              skip_p2p_entry = 1;
              current_state  = STATE_AUTO;
              start_move_deg(0.0f);
            } else {
              /* First time: run homing sequence via sensor */
              motor_speed_cmd = 0.0f;
              control_reset();
              homing_exti_enable();
              current_state = STATE_HOMING_FAST;
            }
            mb_slave.registers[0x01] = 0;
          } else if (mode_cmd & 0x02) {  // MANUAL_MB or sequence trigger
            control_reset();
            motor_speed_cmd = 0.0f;
            /* BaseSystem sends reg[0x01]=2 to start pick-place sequence too.
               If sequence data is loaded (reg[0x22]>0) and autostart flag set
               (reg[0x04] bit0), go straight to SEQUENCE; otherwise jog mode. */
            if (homed && mb_slave.registers[0x22] > 0 && (mb_slave.registers[0x04] & 0x01)) {
              seq_mb_state    = SEQ_MB_IDLE;
              seq_mb_pair_idx = 0;
              seq_mb_timer    = 0;
              seq_mb_step     = 0;
              mb_slave.registers[0x04] &= ~0x01u; /* clear autostart — one-shot */
              current_state   = STATE_SEQUENCE;
            } else {
              current_state   = STATE_MANUAL_MB;
            }
            mb_slave.registers[0x01] = 0;
          } else if (mode_cmd & 0x04) {  // AUTO — requires homed
            if (homed) { skip_p2p_entry = 0; current_state = STATE_AUTO; seq_mb_state = SEQ_MB_IDLE; seq_mb_pair_idx = 0; }
            mb_slave.registers[0x01] = 0;
          } else if (mode_cmd & 0x08) {  // SET HOME — same as RT+A
            dev_dash.Cmd.set_home = 1;
            mb_slave.registers[0x01] = 0;
          } else if (mode_cmd & 0x10) {  // TEST
            if (homed) { current_state = STATE_TEST; test_state = TEST_GOING_END; test_mv_armed = 1; test_repeat = (int16_t)mb_slave.registers[0x11]; }
            mb_slave.registers[0x01] = 0;
          }
        }

        // Cancel / Emergency Stop — หยุดทันที + STATE_EMER + ต้องกด RESET ถึงสั่งใหม่ได้
        if (dev_dash.Cmd.cancel_move) {
          dev_dash.Cmd.cancel_move = 0;
          motor_speed_cmd          = 0.0f;
          control_reset();
          // Atomic: ป้องกัน ISR override PWM ระหว่าง transition
          __disable_irq();
          current_state = STATE_EMER;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
          __enable_irq();
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
          /* RESET_LED and TOWER_R handled by tower lights */
        }

        // ── X button: EMER from any non-EMER state ────────────────────────────
        if (joy_is_connected() && joy_btn(BTN_X) && current_state != STATE_EMER) {
          motor_speed_cmd = 0.0f;
          control_reset();
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
          current_state = STATE_EMER;
        }

        // ── Execute State Machine ─────────────────────────────────────────────
        switch (current_state) {
          case STATE_IDLE:
            mb_slave.registers[0x27] = 0;
            /* After homing via LT+A: wait 1s then drive to SET HOME position */
            if (joy_is_connected()) {
              if (joy_rt_f() > 0.5f) {
                motor_speed_cmd = joy_ly_f();              // normal speed
              } else if (joy_btn(BTN_DPAD_LEFT)) {
                motor_speed_cmd =  (homing_slow_speed > 0.01f ? homing_slow_speed : 0.01f);
              } else if (joy_btn(BTN_DPAD_RIGHT)) {
                motor_speed_cmd = -(homing_slow_speed > 0.01f ? homing_slow_speed : 0.01f);
              } else {
                motor_speed_cmd = 0.0f;
              }
              /* X=EMER handled globally above */
              /* LT+LB = HOME (via proximity sensor always) */
              if (joy_lt_f() > 0.5f && joy_btn(BTN_LB)) { homing_exti_enable(); current_state = STATE_HOMING_FAST; }
              // T = enter P2P AUTO mode (requires homed)
              static uint8_t t_prev_idle = 0;
              if (joy_btn(BTN_T) && !t_prev_idle && homed) current_state = STATE_AUTO;
              t_prev_idle = joy_btn(BTN_T);
            } else {
              motor_speed_cmd = 0.0f;
              joy_seq_state = JSEQ_IDLE;
            }
            break;
            
          case STATE_HOMING_FAST:
            mb_slave.registers[0x27] = 0x01;
            motor_speed_cmd = -homing_fast_speed; // direct PWM left
            if (homing_sensor_flag) {
              homing_sensor_flag = 0;
              motor_speed_cmd    = 0.0f;
              current_state      = STATE_HOMING_BACKOFF;
            }
            /* X=EMER handled globally */
            break;

          case STATE_HOMING_BACKOFF: {
            mb_slave.registers[0x27] = 0x01;
            static uint32_t back_tick = 0;
            motor_speed_cmd = homing_backoff_speed; // direct PWM right (no PID)
            if (++back_tick >= homing_backoff_ticks) {
              back_tick       = 0;
              motor_speed_cmd = 0.0f;
              homing_exti_enable();
              current_state   = STATE_HOMING_SLOW;
            }
            /* X=EMER handled globally */
            break;
          }

          case STATE_HOMING_SLOW:
            mb_slave.registers[0x27] = 0x01;
            motor_speed_cmd = -homing_slow_speed; // direct PWM very slow left
            if (homing_sensor_flag) {
              homing_sensor_flag = 0;
              motor_speed_cmd    = 0.0f;
              finish_homing();   // zeros encoder, resets cumulative, state→IDLE
            }
            /* X=EMER handled globally */
            break;
            
          case STATE_MANUAL: {
            /* Cabinet switch HIGH: joystick only, no PID, no Modbus motor control.
               RT (deadman) + LY = motor max 60%,  DPAD = ±homing_slow_speed fine tune
               Y(edge) = toggle cylinder,  B(edge) = toggle gripper
               LT+LB(edge) = start homing sequence (same as IDLE)
               LB(edge) = pick seq,  RB(edge) = place seq
               RT+A(edge) = set home  (also caught by global set_home block above)
               X = EMER  (caught by global X=EMER block above)               */
            mb_slave.registers[0x27] = 0;

            static float  man_ramp     = 0.0f;
            static uint8_t man_cyl     = 0;   /* 0=up  1=down */
            static uint8_t man_grip    = 0;   /* 0=open 1=closed */
            static uint8_t y_prev_man  = 0, b_prev_man = 0;
            static uint8_t lb_prev_man = 0, rb_prev_man = 0;

            if (joy_is_connected()) {
              uint8_t rt_held = (joy_rt_f() > 0.5f);
              uint8_t btn_y   = joy_btn(BTN_Y);
              uint8_t btn_b   = joy_btn(BTN_B);
              uint8_t btn_lb  = joy_btn(BTN_LB);
              uint8_t btn_rb  = joy_btn(BTN_RB);

              /* RT + LY: ramp toward target, max 0.6 */
              float tgt = rt_held ? (joy_ly_f() * 0.6f) : 0.0f;
              if (fabsf(tgt) < 0.08f) tgt = 0.0f;  /* dead zone */
              const float MAN_RAMP = 0.03f;
              if      (tgt > man_ramp + MAN_RAMP) man_ramp += MAN_RAMP;
              else if (tgt < man_ramp - MAN_RAMP) man_ramp -= MAN_RAMP;
              else                                 man_ramp  = tgt;
              motor_speed_cmd = man_ramp;

              /* DPAD fine tune: ±homing_slow_speed (tunable via Live Expressions) */
              if      (joy_btn(BTN_DPAD_RIGHT)) motor_speed_cmd =  homing_slow_speed;
              else if (joy_btn(BTN_DPAD_LEFT))  motor_speed_cmd = -homing_slow_speed;

              /* Y: toggle cylinder */
              if (btn_y && !y_prev_man) {
                man_cyl ^= 1;
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin,
                                  man_cyl ? GPIO_PIN_SET : GPIO_PIN_RESET);
              }

              /* B: toggle gripper */
              if (btn_b && !b_prev_man) {
                man_grip ^= 1;
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin,
                                  man_grip ? GPIO_PIN_SET : GPIO_PIN_RESET);
              }

              /* LT+LB: start homing sequence (same combo as IDLE) */
              if (joy_lt_f() > 0.5f && btn_lb && !lb_prev_man) {
                man_ramp        = 0.0f;
                motor_speed_cmd = 0.0f;
                control_reset();
                homing_exti_enable();
                current_state = STATE_HOMING_FAST;
                lb_prev_man = btn_lb;
                break;
              }

              /* LB alone: start pick sequence (open gripper + down) */
              if (btn_lb && !lb_prev_man && act_seq_state == ACT_IDLE) {
                HAL_GPIO_WritePin(GRIPPER_GPIO_Port,   GRIPPER_Pin,   GPIO_PIN_RESET); /* open */
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);   /* down */
                act_seq_state = ACT_PK_DOWN;
                act_seq_timer = 0;
              }
              /* RB: start place sequence (down with gripper closed) */
              if (btn_rb && !rb_prev_man && act_seq_state == ACT_IDLE) {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);   /* down */
                act_seq_state = ACT_PL_DOWN;
                act_seq_timer = 0;
              }

              y_prev_man  = btn_y;
              b_prev_man  = btn_b;
              lb_prev_man = btn_lb;
              rb_prev_man = btn_rb;

            } else {
              /* Joystick disconnected: ramp to zero */
              if      (man_ramp >  0.01f) man_ramp -= 0.05f;
              else if (man_ramp < -0.01f) man_ramp += 0.05f;
              else                        man_ramp  = 0.0f;
              motor_speed_cmd = man_ramp;
            }

            /* If cabinet switch was flipped back to LOW, transition handled by mode switch handler */
            break;
          }

          case STATE_MANUAL_MB:
            /* Modbus MANUAL tab: jog handled globally, actuator handled globally.
               Just allow mode transitions from here. */
            mb_slave.registers[0x27] = 0;
            /* Mode transitions handled by global reg[0x01] handler above.
               Motor output = 0 (jog handled globally, sets STATE_AUTO). */
            motor_speed_cmd = 0.0f;
            break;

          case STATE_AUTO: {
            uint8_t traj_done = ctrl_settled;

            // If sequence condition is now met, hand off to STATE_SEQUENCE
            if (mb_slave.registers[0x22] > 0 && (mb_slave.registers[0x04] & 0x01)) {
              seq_mb_state    = SEQ_MB_IDLE;
              seq_mb_pair_idx = 0;
              seq_mb_timer    = 0;
              seq_mb_step     = 0;
              mb_slave.registers[0x04] &= ~0x01u; /* clear autostart — one-shot */
              current_state   = STATE_SEQUENCE;
              break;
            }

            mb_slave.registers[0x27] = 0x08;
            {
              int16_t p2p_raw  = (int16_t)mb_slave.registers[0x24];
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

            // Joystick in AUTO: T=abort+hold→IDLE, DPAD L/R=±5° nudge, X=EMER(global)
            if (joy_is_connected()) {
              static uint8_t t_prev_auto = 0;
              if (joy_btn(BTN_T) && !t_prev_auto) {
                float hold = my_encoder.position_rad;
                ctrl_direct_target = hold;
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

            /* Reed confirm: wait dwell_ms minimum, then check reed; OR force after timeout.
               When reed_dummy_en=1 the reed_ variables mirror output automatically. */
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
                /* step 0: wait reed_down + dwell → close gripper
                   step 1: wait reed_grip + dwell → cylinder UP
                   step 2: wait reed_up  + dwell → go to GOING_PLACE */
                seq_mb_timer++;
                switch (seq_mb_step) {
                  case 0:
                    if (SEQ_OK(reed_down)) {
                      HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET); /* close */
                      seq_mb_timer = 0; seq_mb_step = 1;
                    }
                    break;
                  case 1:
                    if (SEQ_OK(reed_grip)) {
                      HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); /* UP */
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
                /* step 0: wait reed_down + dwell → open gripper
                   step 1: wait !reed_grip + dwell → cylinder UP
                   step 2: wait reed_up  + dwell → next pair */
                seq_mb_timer++;
                switch (seq_mb_step) {
                  case 0:
                    if (SEQ_OK(reed_down)) {
                      HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET); /* open */
                      seq_mb_timer = 0; seq_mb_step = 1;
                    }
                    break;
                  case 1:
                    if (SEQ_OK(!reed_grip)) {
                      HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); /* UP */
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
                        current_state = STATE_AUTO;
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
              dev_dash.Ctrl.max_velocity = tst_v;  /* propagates to control_model[0] via 100Hz sync */
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
            break; // handled by common emer_active block above
            
          default:
            current_state = STATE_IDLE;
            break;
        }
        
        // Modbus Actuator Control (BaseSystem register map) — runs in AUTO regardless of joystick
        {
          // 0x02: Manual actuator — only when no actuator sequence is running
          // 0xFFFF sentinel: first write (including 0x00=UP) always triggers
          if (seq_mb_state == SEQ_MB_IDLE && act_seq_state == ACT_IDLE) {
            static uint16_t prev_grip_cmd = 0xFFFF;
            uint16_t grip_cmd = mb_slave.registers[0x02];
            if (grip_cmd != prev_grip_cmd) {
              // BaseSystem sends one-hot values: 0x01=DOWN, 0x00=UP, 0x02=OPEN, 0x04=CLOSE
              // If new value has gripper bits set → gripper-only command, leave cylinder alone
              // If new value has NO gripper bits → cylinder command (bit0 = state)
              if (grip_cmd & 0x06) {
                if (grip_cmd & 0x02) HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET); // OPEN
                if (grip_cmd & 0x04) HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);   // CLOSE
              } else {
                HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin,
                                  (grip_cmd & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
              }
              prev_grip_cmd = grip_cmd;
            }
          }

          // 0x03: Simple pick/place actuator sequence (bit0=Pick, bit1=Place)
          // pick:  open → down → wait reed_down → close → wait reed_grip → up
          // place: down → wait reed_down → open → wait !reed_grip → up
          {
            static uint16_t seq3_prev = 0;
            uint16_t seq3 = mb_slave.registers[0x03];
            /* Korn: auto-clear register after any non-zero, rising-edge only */
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
            if (seq3 != 0u) {  /* Korn: firmware auto-clears 0x03 after fire */
              mb_slave.registers[0x03] = 0u;
              seq3_prev = 0u;
            } else {
              seq3_prev = seq3;
            }

            // Run actuator state machine: use seq_dwell_ms (min wait) / seq_timeout_ms (force)
            act_seq_timeout = (uint32_t)(seq_dwell_ms / 10.0f);
            if (act_seq_timeout < 1) act_seq_timeout = 1;

            switch (act_seq_state) {
              case ACT_IDLE: break;

              // Wait seq_dwell_ms then confirm with reed; force-advance at seq_timeout_ms
              #define ACT_OK(reed) ((act_seq_timer >= act_seq_timeout && (reed)) || \
                                    act_seq_timer >= (uint32_t)(seq_timeout_ms / 10.0f))
              case ACT_PK_DOWN:
                act_seq_timer++;
                if (ACT_OK(reed_down)) {
                  HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET); // close
                  act_seq_state = ACT_PK_GRIP;
                  act_seq_timer = 0;
                }
                break;

              case ACT_PK_GRIP:
                act_seq_timer++;
                if (ACT_OK(reed_grip)) {
                  HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); // up
                  act_seq_state = ACT_PK_UP;
                  act_seq_timer = 0;
                }
                break;

              case ACT_PK_UP:
                act_seq_timer++;
                if (ACT_OK(reed_up)) {
                  mb_slave.registers[0x27] |= 0x02; // pick done
                  act_seq_state = ACT_IDLE;
                }
                break;

              case ACT_PL_DOWN:
                act_seq_timer++;
                if (ACT_OK(reed_down)) {
                  HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET); // open
                  act_seq_state = ACT_PL_OPEN;
                  act_seq_timer = 0;
                }
                break;

              case ACT_PL_OPEN:
                act_seq_timer++;
                if (ACT_OK(!reed_grip)) {
                  HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); // up
                  act_seq_state = ACT_PL_UP;
                  act_seq_timer = 0;
                }
                break;

              case ACT_PL_UP:
                act_seq_timer++;
                if (ACT_OK(reed_up)) {
                  mb_slave.registers[0x27] |= 0x04; // place done
                  act_seq_state = ACT_IDLE;
                }
                break;
              #undef ACT_OK

              default: act_seq_state = ACT_IDLE; break;
            }
          }
        }
        
        // Apply Motor Command — PID states handle their own PWM in TIM7 ISR
        if (current_state != STATE_AUTO &&
            current_state != STATE_SEQUENCE &&
            current_state != STATE_TEST) {
          static float prod_ramp = 0.0f;
          static uint8_t prod_dead_cnt = 0;
          float prod_raw = motor_speed_cmd;
          if (current_state == STATE_EMER) prod_raw = 0.0f;
          /* Soft limit directional clamp for non-PID states */
          if (soft_limit_dir > 0 && prod_raw > 0.0f) prod_raw = 0.0f;
          if (soft_limit_dir < 0 && prod_raw < 0.0f) prod_raw = 0.0f;
          float prod_target = (prod_raw >  dev_dash.Sys.max_speed) ?  dev_dash.Sys.max_speed :
                              (prod_raw < -dev_dash.Sys.max_speed) ? -dev_dash.Sys.max_speed : prod_raw;
          float prod_ramp_rate_local = dev_dash.Sys.ramp_rate;
          if ((prod_target > 0.0f && prod_ramp < 0.0f) ||
              (prod_target < 0.0f && prod_ramp > 0.0f)) {
            prod_dead_cnt = 5;
          }
          if (prod_dead_cnt > 0) { prod_ramp = 0.0f; prod_dead_cnt--; }
          else if (prod_target > prod_ramp + prod_ramp_rate_local) prod_ramp += prod_ramp_rate_local;
          else if (prod_target < prod_ramp - prod_ramp_rate_local) prod_ramp -= prod_ramp_rate_local;
          else                                                      prod_ramp  = prod_target;
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

      } // End Normal Production Mode
      } // End !emer_active

      // ── Tower Lights + RESET_LED (100Hz tick) ────────────────────────────────
      if (dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {
        static uint8_t tl_tick = 0;
        tl_tick++;

        uint8_t G = 0, Y = 0, R = 0;

        if (emer_active) {
          uint8_t estop_pressed = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET);
          if (estop_pressed) {
            R = (tl_tick % 50) < 25 ? 1 : 0;  /* ESTOP held: R blink 2Hz */
          } else {
            R = 1;                              /* ESTOP released: R steady (wait RESET) */
          }
          /* RESET_LED: off when ESTOP pressed (can't exit), blink when waiting for RESET */
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin,
            estop_pressed ? GPIO_PIN_RESET
                          : ((tl_tick % 50) < 25 ? GPIO_PIN_SET : GPIO_PIN_RESET));

        } else {
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_RESET); /* off when not EMER */

          if (current_state == STATE_HOMING_FAST ||
              current_state == STATE_HOMING_BACKOFF ||
              current_state == STATE_HOMING_SLOW) {
            Y = 0; G = 0; R = 0; /* tower off during homing — relay noise */

          } else if (!homed) {
            /* Not homed: Y↔G alternating blink — prompt to home */
            Y = (tl_tick % 50) < 25 ? 1 : 0;
            G = (tl_tick % 50) < 25 ? 0 : 1;

          } else if (current_state == STATE_IDLE || current_state == STATE_MANUAL ||
                     current_state == STATE_MANUAL_MB) {
            if (sethome_led_cnt > 0) {
              /* Set-home flash: Y · Y · G */
              uint8_t c = sethome_led_cnt;
              Y = (c > 100) || (c > 60 && c <= 80) ? 1 : 0;
              G = (c <= 40) ? 1 : 0;
              if (++sethome_led_cnt > 130) sethome_led_cnt = 0;
            } else if (current_state == STATE_MANUAL || current_state == STATE_MANUAL_MB) {
              Y = 1;                             /* MANUAL: Y steady */
            } else {
              G = 1;                             /* IDLE + homed: G steady */
            }

          } else if (current_state == STATE_AUTO) {
            /* Velocity-proportional blink */
            float spd = fabsf(ctrl_vel_rad_s);
            if      (spd > 4.0f) G = (tl_tick % 10) < 5 ? 1 : 0;   /* 5Hz */
            else if (spd > 1.0f) G = (tl_tick % 30) < 15 ? 1 : 0;  /* 1.7Hz */
            else if (spd > 0.2f) G = (tl_tick % 60) < 30 ? 1 : 0;  /* slow */
            else                 G = 1;                               /* settled: steady */

          } else if (current_state == STATE_SEQUENCE) {
            G = (tl_tick % 50) < 25 ? 1 : 0;   /* G blink 2Hz */
            Y = 1;                               /* Y steady */

          } else if (current_state == STATE_TEST) {
            G = 1;
            Y = (tl_tick % 50) < 25 ? 1 : 0;   /* Y blink 2Hz */
          }

          /* Soft limit overlay: R+G steady (warns of rotation limit) */
          if (soft_limit_dir != 0) {
            R = 1; G = 1; Y = 0;
          }
        }

        HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, G ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, Y ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, R ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      // 7. Update Live Expressions Dashboard
            dev_dash.IO.joy_connected = joy_is_connected();
            dev_dash.IO.joy_buttons   = joy_raw_buttons();
            dev_dash.IO.joy_ly       = joy_ly_f();
            dev_dash.IO.joy_rt       = joy_rt_f();
            dev_dash.IO.joy_lt       = joy_lt_f();

            dev_dash.Status.status_state         = current_state;
            dev_dash.Status.motor_cmd     = (current_state == STATE_AUTO)
                                            ? motor_speed_cmd
                                            : ((HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_SET) ? safe_speed : -safe_speed);
            dev_dash.Status.encoder_raw       = current_position;
            dev_dash.Status.current_A     = current_sensor_A;
            dev_dash.Status.pos_rad       = current_position * RAD_PER_CNT;
            { // pos_deg: 0–360° wrapped (8192 counts = 1 rev)
              int32_t pos_mod = (int32_t)current_position % 8192;
              if (pos_mod < 0) pos_mod += 8192;
              dev_dash.Status.pos_deg = (float)pos_mod * (360.0f / 8192.0f);
            }
            dev_dash.Status.vel_rad_s     = ctrl_vel_rad_s;
            dev_dash.Status.acc_rad_s2    = ctrl_acc_rad_s2;

            dev_dash.IO.in_estop      = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin);
            dev_dash.IO.in_mode = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
            dev_dash.IO.in_reset      = HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin);
            dev_dash.IO.in_power      = HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin);

            dev_dash.IO.out_pwm         = (safe_speed > 0.0f || safe_speed < 0.0f) ? 1 : 0;
            dev_dash.IO.out_dir         = HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin);
            dev_dash.IO.out_power_latch = HAL_GPIO_ReadPin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin);
            dev_dash.IO.out_pneumatic   = HAL_GPIO_ReadPin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin);
            dev_dash.IO.out_gripper     = HAL_GPIO_ReadPin(GRIPPER_GPIO_Port, GRIPPER_Pin);
            dev_dash.IO.reed_up         = reed_up;
            dev_dash.IO.reed_down       = reed_down;
            dev_dash.IO.reed_grip       = reed_grip;
            dev_dash.IO.out_tower_g   = HAL_GPIO_ReadPin(TOWER_G_GPIO_Port, TOWER_G_Pin);
            dev_dash.IO.out_tower_y   = HAL_GPIO_ReadPin(TOWER_Y_GPIO_Port, TOWER_Y_Pin);
            dev_dash.IO.out_tower_r   = HAL_GPIO_ReadPin(TOWER_R_GPIO_Port, TOWER_R_Pin);
            dev_dash.IO.out_reset_led       = HAL_GPIO_ReadPin(RESET_LED_GPIO_Port, RESET_LED_Pin);
            dev_dash.IO.out_emer            = HAL_GPIO_ReadPin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin);
            dev_dash.Sys.telemetry_mode = dev_dash.Sys.telemetry_mode;

      // Send telemetry @ 50Hz (ทุก 2 รอบ) — 19200 baud/16 bytes ≈ 9ms ต้องการ gap ≥ 20ms
      static uint8_t telem_div = 0;
      if (enable_telemetry && ++telem_div >= 2) { telem_div = 0; Send_Telemetry(); }

      // Refresh IWDG
      HAL_IWDG_Refresh(&hiwdg);
      
      // Heartbeat LED
      static uint32_t last_hb = 0;
      if (++last_hb > 50) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        last_hb = 0;
      }
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 25;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* Accept all standard + extended frames into RX FIFO0 (no ID filter needed) */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
      FDCAN_ACCEPT_IN_RX_FIFO0,   /* non-matching std frames  → FIFO0 */
      FDCAN_ACCEPT_IN_RX_FIFO0,   /* non-matching ext frames  → FIFO0 */
      FDCAN_FILTER_REMOTE,
      FDCAN_FILTER_REMOTE);

  HAL_FDCAN_Start(&hfdcan1);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 19200;   /* BaseSystem v1.1 */
  hlpuart1.Init.WordLength = UART_WORDLENGTH_9B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_EVEN;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 169;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PNEUMATIC_Pin|GRIPPER_Pin|POWER_LATCH_Pin|TOWER_R_Pin
                          |TOWER_Y_Pin|TOWER_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_BTN_Pin MODE_Pin HOME_SENSOR_Pin */
  GPIO_InitStruct.Pin = RESET_BTN_Pin|MODE_Pin|HOME_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : REED_DOWN_Pin REED_UP_Pin */
  GPIO_InitStruct.Pin = REED_DOWN_Pin|REED_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : REED_GRIP_Pin */
  GPIO_InitStruct.Pin = REED_GRIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(REED_GRIP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PNEUMATIC_Pin GRIPPER_Pin POWER_LATCH_Pin TOWER_R_Pin
                           TOWER_Y_Pin TOWER_G_Pin */
  GPIO_InitStruct.Pin = PNEUMATIC_Pin|GRIPPER_Pin|POWER_LATCH_Pin|TOWER_R_Pin
                          |TOWER_Y_Pin|TOWER_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_Pin EMER_OUTPUT_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin|EMER_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_LED_Pin */
  GPIO_InitStruct.Pin = RESET_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_BTN_Pin */
  GPIO_InitStruct.Pin = POWER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_BTN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ── Telemetry Packet ─────────────────────────────────────────────────────────
// Format (24 bytes):
//   [0x7E][0x7E] [pos:f32] [vel:f32] [acc:f32] [pos_ref:f32] [vel_ref:f32] [0x03][0x03]
//
// Python: Serial Receive → 24 bytes → unpack 5×float
//
void Send_Telemetry(void)
{
  uint8_t buf[24];
  float pos     = (float)(current_position) * RAD_PER_CNT;
  float vel     = (float)ctrl_vel_rad_s;
  float acc     = (float)ctrl_acc_rad_s2;
  float pos_ref = dev_dash.Status.pos_ideal;
  float vel_ref = dev_dash.Status.vel_ideal;

  buf[0]  = 0x7E;
  buf[1]  = 0x7E;
  memcpy(&buf[2],  &pos,     4);
  memcpy(&buf[6],  &vel,     4);
  memcpy(&buf[10], &acc,     4);
  memcpy(&buf[14], &pos_ref, 4);
  memcpy(&buf[18], &vel_ref, 4);
  buf[22] = 0x03;
  buf[23] = 0x03;

  // LPUART1 19200 8E1 — 24 bytes = ~13.7ms, timeout 20ms
  HAL_UART_Transmit(&hlpuart1, buf, sizeof(buf), 20);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // ESTOP: polled in 100Hz loop (not interrupt-based, anti-EMI)

  if (GPIO_Pin == HOME_SENSOR_Pin) {
    // Only act during homing states
    if (current_state != STATE_HOMING_FAST && current_state != STATE_HOMING_SLOW) return;
    // Debounce: set pending flag, 1kHz ISR will validate after 1ms
    homing_sensor_pending = 1;
    // Disable EXTI immediately to prevent re-trigger during debounce window
    __HAL_GPIO_EXTI_CLEAR_IT(HOME_SENSOR_Pin);
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
    // ── 1kHz Control Loop ────────────────────────────────────────────────────
    uint16_t enc_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    if (enc_reset_pending) {
      enc_reset_pending = 0;
      Encoder_Init(&my_encoder, LOOP_DT);
      my_encoder.last_counter_value = enc_now;
      current_position = 0;
      // cumulative_angle_deg is reset only by finish_homing(), not by set_home
    }

    // Encoder direction — set encoder_inverted=1 in Live Expressions if position goes wrong way
    {
      int16_t hw_delta = (int16_t)(enc_now - my_encoder.last_counter_value);
      int32_t delta    = encoder_inverted ? (int32_t)hw_delta : -(int32_t)hw_delta;
      my_encoder.last_counter_value = enc_now;
      my_encoder.count_accum       += delta;
      my_encoder.position_rad       = (float)my_encoder.count_accum * COUNTS_TO_RAD;
      current_position              = my_encoder.count_accum;
      robot_pos_rad                 = my_encoder.position_rad; /* used by control.c */

      my_encoder.vel_buf[my_encoder.vel_idx] = my_encoder.count_accum;
      if (++my_encoder.vel_idx >= VEL_WINDOW) { my_encoder.vel_idx = 0; my_encoder.vel_full = 1; }
      if (my_encoder.vel_full) {
        int32_t oldest = my_encoder.vel_buf[my_encoder.vel_idx];
        my_encoder.velocity_rad_s = (float)(my_encoder.count_accum - oldest)
                                  * COUNTS_TO_RAD / ((float)VEL_WINDOW * LOOP_DT);
      } else {
        my_encoder.velocity_rad_s = (float)delta * COUNTS_TO_RAD / LOOP_DT;
      }
      // Track cumulative rotation from home (only after homed)
      if (homed && !enc_reset_pending) {
        cumulative_angle_deg += (float)delta * COUNTS_TO_RAD * (180.0f / 3.14159265f);
      }
      // Soft limit ±540°: set directional lock (main loop also reads this)
      if (homed) {
        if      (cumulative_angle_deg >=  540.0f) soft_limit_dir =  1;
        else if (cumulative_angle_deg <= -540.0f) soft_limit_dir = -1;
        else                                       soft_limit_dir =  0;
      }
    }

    // Homing sensor debounce: validate 1ms after EXTI fires
    if (homing_sensor_pending) {
      homing_sensor_pending = 0;
      if (HAL_GPIO_ReadPin(HOME_SENSOR_GPIO_Port, HOME_SENSOR_Pin) == HOME_SENSOR_ACTIVE) {
        homing_sensor_flag = 1; // confirmed, not a glitch
      }
    }

    // IIR velocity filter (reduces quantization noise beyond windowing)
    float vel_alpha = dev_dash.Sys.acc_alpha;
    if (vel_alpha < 0.01f) vel_alpha = 0.01f;
    if (vel_alpha > 1.0f)  vel_alpha = 1.0f;
    vel_filtered   = vel_alpha * my_encoder.velocity_rad_s + (1.0f - vel_alpha) * vel_filtered;
    ctrl_vel_rad_s = vel_filtered;

    // Acceleration LPF
    static float prev_vel_for_acc = 0.0f;
    float raw_acc = (ctrl_vel_rad_s - prev_vel_for_acc) * 1000.0f;
    prev_vel_for_acc = ctrl_vel_rad_s;
    ctrl_acc_rad_s2 = ctrl_acc_rad_s2 * (1.0f - vel_alpha) + raw_acc * vel_alpha;

    // ── AUTO CONTROL @ 1kHz — delegated to control.c ────────────────────────
    if ((current_state == STATE_AUTO     || current_state == STATE_SEQUENCE ||
         current_state == STATE_TEST)
        && dev_dash.Sys.mode == SYS_MODE_PRODUCTION) {

      float pwm_out = 0.0f;
      /* Due uses raw windowed encoder velocity (not IIR-filtered) for PID feedback */
      control_update(robot_pos_rad, my_encoder.velocity_rad_s, &pwm_out);

      /* Motor direction invert */
      if (motor_dir_inverted) pwm_out = -pwm_out;

      float spd      = pwm_out;
      float min_frac = min_pwm_pct / 100.0f;
      if (spd >  dev_dash.Sys.max_speed) spd =  dev_dash.Sys.max_speed;
      if (spd < -dev_dash.Sys.max_speed) spd = -dev_dash.Sys.max_speed;
      uint32_t pwm_cnt;
      if (spd > 0.02f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        if (spd < min_frac) spd = min_frac;
        pwm_cnt = (uint32_t)(spd * (float)htim3.Init.Period);
      } else if (spd < -0.02f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        spd = -spd;
        if (spd < min_frac) spd = min_frac;
        pwm_cnt = (uint32_t)(spd * (float)htim3.Init.Period);
      } else {
        pwm_cnt = 0;
      }
      /* Soft limit directional clamp */
      if (soft_limit_dir > 0 && HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_SET)
        pwm_cnt = 0;
      if (soft_limit_dir < 0 && HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_RESET)
        pwm_cnt = 0;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_cnt);
      motor_speed_cmd = pwm_out;

      dev_dash.Status.pos_rad    = robot_pos_rad;
      dev_dash.Status.vel_rad_s  = vel_filtered;
      dev_dash.Status.pos_ideal  = ctrl_ideal_pos;
      dev_dash.Status.vel_ideal  = ctrl_ideal_vel;
      dev_dash.Status.pos_err    = ctrl_pos_err;
      dev_dash.Status.vel_sp     = ctrl_vel_sp;
      dev_dash.Status.pwm_out    = motor_speed_cmd;
      dev_dash.Status.traj_active = !ctrl_settled;
    }
    // ── END AUTO CONTROL ─────────────────────────────────────────────────────
    // Safety: ถ้าเพิ่งออกจาก STATE_AUTO → force PWM=0 ทันทีใน ISR tick ถัดไป
    {
      static uint8_t isr_was_auto = 0;
      if (current_state == STATE_AUTO     || current_state == STATE_SEQUENCE ||
          current_state == STATE_TEST) {
        isr_was_auto = 1;
      } else if (isr_was_auto) {
        isr_was_auto = 0;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // force PWM=0
      }
    }

    // Poll Power Button
    if (HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin) == GPIO_PIN_RESET) {
      power_btn_hold_ms++;
      if (power_btn_hold_ms >= 3000) {
        HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_RESET); // Shutdown
      }
    } else {
      power_btn_hold_ms = 0;
    }

    // Software Receiver Timeout for Modbus
    modbus_tick_1ms(&mb_slave);

    // 100Hz Sub-loop trigger
    sub_loop_counter++;
    if (sub_loop_counter >= 10) {
      sub_loop_counter = 0;
      flag_10ms = 1;
    }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
