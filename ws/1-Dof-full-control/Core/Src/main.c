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
#include "joystick.h"
#include "modbus.h"
#include "SCURVE.h"
#include "TRAPEZOID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEL_WIN      10                     // velocity window size (ticks)
#define RAD_PER_CNT  (6.28318530718f / 8192.0f)  // counts → radians (2048PPR × 4x)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

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
uint8_t joy_dma_buf[PKT_LEN * 2];
ModbusSlave_t mb_slave;

// Telemetry UART4 (PB9=TX, PB8=RX) → Simulink
UART_HandleTypeDef huart4;

typedef enum {
  STATE_INIT,
  STATE_IDLE,
  STATE_CALIBRATE,
  STATE_MANUAL,
  STATE_AUTO,
  STATE_EMER
} RobotState_t;

RobotState_t current_state = STATE_INIT;

// Motor & PID Globals (For Control Team)
volatile float   motor_speed_cmd  = 0.0f; // -1.0 to 1.0
volatile int32_t current_position = 0;
volatile float   current_sensor_A = 0.0f;

// Power Latch Timer
uint32_t power_btn_hold_ms = 0;

// Auto Control globals
SCurve_t    g_scurve;
Trapezoid_t g_trapezoid;
volatile float ctrl_vel_rad_s  = 0.0f; // เขียน ISR, อ่าน main loop
float ctrl_traj_start          = 0.0f;
float ctrl_direct_target       = 0.0f;
float ctrl_vel_integral        = 0.0f;
float ctrl_pos_integral        = 0.0f;
volatile uint8_t enc_reset_pending = 0;
volatile float ctrl_acc_rad_s2 = 0.0f; // เขียน ISR, อ่าน main loop
uint8_t sethome_led_cnt = 0;           // SET HOME LED feedback counter (@100Hz)

// UI Sub-loop Counter
uint8_t sub_loop_counter = 0;
volatile uint8_t flag_10ms = 0;

// --- Live Expressions Dashboard & UI Testing ---
typedef enum {
  SYS_MODE_PRODUCTION = 0,
  SYS_MODE_HARDWARE_TEST = 1,
  SYS_MODE_JOYSTICK_TEST = 2,
  SYS_MODE_AUTO_MOTOR_TEST = 3
} SystemMode_t;

typedef struct {
  SystemMode_t mode;
  uint8_t  force_pneumatic;
  uint8_t  force_gripper;
  uint8_t  force_tower_green;
  uint8_t  force_tower_yellow;
  uint8_t  force_tower_red;
  uint8_t  force_emer_out;
  float    force_motor_speed;
  // Motor tuning (ปรับ real-time ผ่าน Live Expressions)
  float    ramp_rate;      // slew rate /10ms: 0.01=slow 0.1=fast
  float    max_speed;      // hard cap 0.0-1.0
  // Mode 3: Auto Motor Test
  float    auto_speed;         // speed 0.0-1.0
  uint16_t auto_period_fwd_ms; // ms ทิศ Forward (DIR=SET)
  uint16_t auto_period_rev_ms; // ms ทิศ Reverse (DIR=RESET)
  // Current sensor calibration (WCS1800)
  float    cur_zero_v;     // voltage ที่ 0A (วัดจาก multimeter ตอนไม่มีกระแส)
  float    cur_sens;       // sensitivity V/A (0.066 = 66mV/A)
  uint8_t  telemetry_mode; // 0=Modbus(normal), 1=Telemetry via LPUART1 USB
  float    acc_alpha;      // acceleration LPF alpha (0.0=heavy filter, 1.0=raw)
} DashCtrl_t;

typedef struct {
  uint8_t  connected;
  uint16_t raw_buttons;
  float    L_Y;
  float    R_T;
  float    L_T;
} DashJoy_t;

typedef struct {
  RobotState_t state;
  float    motor_cmd;
  int32_t  encoder;
  float    current_A;
  // Telemetry values (ค่าที่ส่งออก UART ไป Simulink)
  float    pos_rad;    // position (rad)
  float    pos_deg;
  float    vel_rad_s;  // velocity (rad/s)
  float    acc_rad_s2; // acceleration (rad/s²)
} DashStatus_t;

typedef struct {
  uint8_t  estop;
  uint8_t  mode_switch;
  uint8_t  reset;
  uint8_t  power;
} DashIn_t;

typedef struct {
  uint8_t  pwm;
  uint8_t  dir;
  uint8_t  pneumatic;
  uint8_t  gripper;
  uint8_t  tower_g;
  uint8_t  tower_y;
  uint8_t  tower_r;
  uint8_t  reset_led;
  uint8_t  emer;
  uint8_t  telemetry_active; // mirror of Ctrl.telemetry_mode สำหรับดูใน Status
} DashOut_t;

// Auto Control Dashboard (STATE_AUTO tuning via Live Expressions)
typedef struct {
  // --- COMMANDS (write these) ---
  float    target_deg;    // absolute target position (degrees from home)
  uint8_t  start_move;     // set 1 to trigger move, auto-clears
  uint8_t  cancel_move;    // set 1 to stop immediately → STATE_EMER, auto-clears
  uint8_t  set_home;       // set 1 to mark current pos as home (0°), auto-clears
  uint8_t  traj_type;    // 0=Trapezoid, 1=S-Curve
  uint8_t  time_mode;    // 0=constraint-based, 1=time-based
  // Trajectory constraints (time_mode=0)
  float    v_max;        // max velocity (rad/s)
  float    a_max;        // max acceleration (rad/s²)
  float    j_max;        // max jerk (rad/s³), S-Curve only
  // Trajectory times (time_mode=1)
  float    t1_seg;       // S-Curve: jerk duration (s)
  float    t2_seg;       // S-Curve: const-accel duration (s)
  float    t_acc_seg;    // Trapezoid: accel duration (s)
  float    t_cruise_seg; // cruise duration (s)
  // PID gains
  float    kp_vel;
  float    ki_vel;
  float    kd_vel;
  float    kp_pos;
  float    ki_pos;
  float    kd_pos;
  uint8_t  motor_invert; // set 1 ถ้า motor วิ่งสวนทาง (error เพิ่มแทนที่จะลด)
  // --- STATUS (read these) ---
  float    pos_rad;      // current position (rad)
  float    vel_rad_s;    // current velocity (rad/s)
  float    pos_ideal;    // trajectory reference (rad)
  float    vel_ideal;    // trajectory velocity reference (rad/s)
  float    pos_err;      // position error (rad)
  float    vel_sp;       // velocity setpoint from pos loop (rad/s)
  float    pwm_out;      // final motor command (-1.0 to 1.0)
  uint8_t  traj_active;  // 1 while trajectory is running
} DashAuto_t;

typedef struct {
  DashCtrl_t   Ctrl;
  DashJoy_t    Joy;
  DashStatus_t Status;
  DashIn_t     In;
  DashOut_t    Out;
  DashAuto_t   Auto;
} DevDashboard_t;

DevDashboard_t dev_dash = {
  .Ctrl.mode          = SYS_MODE_PRODUCTION,
  .Ctrl.ramp_rate     = 0.05f,
  .Ctrl.max_speed     = 0.40f,
  .Ctrl.auto_speed         = 0.30f,
  .Ctrl.auto_period_fwd_ms = 1000,
  .Ctrl.auto_period_rev_ms = 1000,
  .Ctrl.cur_zero_v      = 2.5f,
  .Ctrl.cur_sens        = 0.066f,
  .Ctrl.telemetry_mode  = 0,   // 0=Modbus, 1=Simulink via USB
  .Ctrl.acc_alpha       = 0.2f, // acceleration LPF (0.1=smooth, 0.5=fast)
  // Auto control defaults
  .Auto.target_deg    = 0.0f,
  .Auto.traj_type     = 0,       // 0=Trapezoid
  .Auto.time_mode     = 0,       // 0=constraint-based
  .Auto.v_max         = 3.14f,   // π/2 rad/s (~0.25 rev/s)
  .Auto.a_max         = 6.28f,   // π rad/s²
  .Auto.j_max         = 10.0f,   // S-Curve jerk
  .Auto.t1_seg        = 0.314f,
  .Auto.t2_seg        = 0.186f,
  .Auto.t_acc_seg     = 0.500f,
  .Auto.t_cruise_seg  = 1.186f,
  .Auto.kp_vel        = 20.0f,
  .Auto.ki_vel        = 0.0f,   // เริ่มที่ 0 — ค่อยเพิ่มทีละ 0.05 หลัง P ทำงานได้
  .Auto.kd_vel        = 0.0f,
  .Auto.kp_pos        = 1.0f,
  .Auto.ki_pos        = 0.0f,
  .Auto.kd_pos        = 0.0f,
};
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
/* USER CODE BEGIN PFP */
static void UART4_Init(void);
static void Send_Telemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
  // Use HAL_UART_Transmit for debug output (careful with Modbus on same port)
  // For now, let's assume LPUART1 is dedicated to Modbus if UI is active.
  // If user wants debug, we can use another port or conditional compile.
  return ch;
}

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
  /* USER CODE BEGIN 2 */
  // EMER (ESTOP_Pin): active HIGH — pin HIGH = กด, ใช้ PULLDOWN
  // MODE selector: PULLUP (LOW=AUTO, HIGH=MANUAL)
  {
    GPIO_InitTypeDef g = {0};
    g.Pin  = ESTOP_Pin;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN;
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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  // Start the 1kHz Control Loop
  HAL_TIM_Base_Start_IT(&htim7);
  
  // Latch Power On (Commented out as PB14 is now TOWER_R)
  // HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);
  
  // Refresh Watchdog
  HAL_IWDG_Refresh(&hiwdg);
  
  current_state = STATE_IDLE;
  mb_slave.registers[0x00] = 22881; // Initialize Heartbeat (YA)
  
  // Calibrate ADC2 before using it
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  
  printf("\r\n=== 1-DOF Robot System Ready ===\r\n");

  // Init trajectory generators (dt=0.001s for 1kHz ISR)
  Trapezoid_Init(&g_trapezoid, dev_dash.Auto.v_max, dev_dash.Auto.a_max, 0.001f);
  SCurve_Init(&g_scurve, dev_dash.Auto.v_max, dev_dash.Auto.a_max, dev_dash.Auto.j_max, 0.001f);

  // Init Telemetry UART (UART4: PB9=TX, PB8=RX, 115200 8N1)
  UART4_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (flag_10ms) {
      flag_10ms = 0;
      
      // 0. Robust ESTOP Polling (Anti-EMI Noise)
      static uint8_t estop_debounce = 0;
      if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET) {
        if (estop_debounce < 5) estop_debounce++;
        if (estop_debounce >= 3) { // Requires 30ms of solid HIGH signal to trigger
          current_state = STATE_EMER;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_SET);
        }
      } else {
        estop_debounce = 0;
      }
      
      // 1. Read ADC Current Sensor (with Moving Average Filter)
      if (HAL_ADC_Start(&hadc2) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
          uint16_t raw_adc = HAL_ADC_GetValue(&hadc2);
          static float filtered_adc = 0;
          if (filtered_adc == 0) filtered_adc = raw_adc;
          filtered_adc = (filtered_adc * 7.0f + (float)raw_adc) / 8.0f;
          // WCS1800: ปรับ cur_zero_v และ cur_sens ใน dev_dash.Ctrl ได้ real-time
          float v = (filtered_adc / 4095.0f) * 3.3f;
          float i_a = (v - dev_dash.Ctrl.cur_zero_v) / dev_dash.Ctrl.cur_sens;
          if (i_a < 0.0f) i_a = -i_a;
          current_sensor_A = i_a; // Amperes (float, 4 decimal places)
        }
      }

      // 2. Process incoming Modbus frames (Always ON in Multiplexed mode)
      modbus_process(&mb_slave);

      // --- AI Auto-Tuner & Python UI Integration ---
      // แยก Register สำหรับ Python UI ออกมาเพื่อไม่ให้ชนกับระบบเดิม
      // Reg 10: Step CMD, Reg 11: Target Pos, Reg 12: Kp, Reg 13: Ki, Reg 14: Kd, Reg 15: Apply
      static uint8_t python_step_active = 0;
      int16_t step_cmd = (int16_t)mb_slave.registers[10];
      if (step_cmd != 0) {
        dev_dash.Ctrl.mode = SYS_MODE_HARDWARE_TEST;
        dev_dash.Ctrl.force_motor_speed = (float)step_cmd / 10000.0f; 
        python_step_active = 1;
      } else if (python_step_active) {
        dev_dash.Ctrl.mode = SYS_MODE_PRODUCTION;
        dev_dash.Ctrl.force_motor_speed = 0.0f;
        python_step_active = 0;
      }

      if (mb_slave.registers[15] == 1) {
        dev_dash.Auto.kp_vel = (float)mb_slave.registers[12] / 100.0f;
        dev_dash.Auto.ki_vel = (float)mb_slave.registers[13] / 100.0f;
        dev_dash.Auto.kd_vel = (float)mb_slave.registers[14] / 100.0f;
        
        float target_rad = (float)((int16_t)mb_slave.registers[11]) / 1000.0f;
        dev_dash.Auto.target_deg = target_rad * (180.0f / 3.14159265f);
        
        // Trajectory overrides (Registers 16-19)
        if (mb_slave.registers[16] <= 2) {
             dev_dash.Auto.traj_type = (uint8_t)mb_slave.registers[16];
        }
        if (mb_slave.registers[17] > 0) {
             dev_dash.Auto.v_max = (float)mb_slave.registers[17] / 100.0f;
        }
        if (mb_slave.registers[18] > 0) {
             dev_dash.Auto.a_max = (float)mb_slave.registers[18] / 100.0f;
        }
        if (mb_slave.registers[19] > 0) {
             dev_dash.Auto.j_max = (float)mb_slave.registers[19] / 100.0f;
        }

        dev_dash.Auto.start_move = 1;
        
        // Force the State Machine to AUTO so it actually executes the move!
        if (current_state != STATE_EMER) {
            current_state = STATE_AUTO;
        }
        
        mb_slave.registers[15] = 0; // Clear apply flag
      }

      // 3. Update Status Registers (0x26 - 0x31)
      mb_slave.registers[0x26] = (HAL_GPIO_ReadPin(REED_UP_GPIO_Port, REED_UP_Pin) == GPIO_PIN_RESET ? 1 : 0) |
                                 (HAL_GPIO_ReadPin(REED_DOWN_GPIO_Port, REED_DOWN_Pin) == GPIO_PIN_RESET ? 2 : 0) |
                                 (HAL_GPIO_ReadPin(REED_GRIP_GPIO_Port, REED_GRIP_Pin) == GPIO_PIN_RESET ? 4 : 0);
      
      mb_slave.registers[0x28] = (int16_t)current_position; 
      mb_slave.registers[0x31] = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
      mb_slave.registers[0x04] = (uint16_t)(current_sensor_A * 1000.0f); // Modbus เก็บเป็น mA (int)
      
      // Heartbeat Logic
      static uint32_t hb_timer = 0;
      hb_timer++;
      if (mb_slave.registers[0x00] == 18537) {
        // UI replied HI
        hb_timer = 0;
        mb_slave.registers[0x00] = 22881; // Ask YA again
      } else if (hb_timer > 300) { // 3 seconds timeout
        // UI is disconnected
        // Handle disconnection if needed
      }

      // Handle Hardware Selector Switch (LOW=AUTO, HIGH=IDLE)
      // init=2 (invalid) → บังคับ detect ครั้งแรกเสมอ ป้องกัน stuck ที่ IDLE ตอน startup
      static GPIO_PinState last_mode_switch = (GPIO_PinState)2;
      GPIO_PinState current_mode_switch = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
      if (current_mode_switch != last_mode_switch) {
        if (current_state != STATE_EMER && current_state != STATE_CALIBRATE &&
            dev_dash.Ctrl.mode == SYS_MODE_PRODUCTION) {
          current_state = (current_mode_switch == GPIO_PIN_RESET) ? STATE_AUTO : STATE_IDLE;
        }
        last_mode_switch = current_mode_switch;
      }

      // ── EMER: ทำงานทุกโหมด (HW Test, Joy Test, Auto Test, Production) ─────
      float safe_speed = 0.0f; // stays 0 when EMER active
      uint8_t emer_active = (current_state == STATE_EMER);
      if (emer_active) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(TOWER_R_GPIO_Port,     TOWER_R_Pin,     GPIO_PIN_SET);
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RESET_LED_GPIO_Port,   RESET_LED_Pin,   GPIO_PIN_SET);
        mb_slave.registers[0x27] = 0;
        motor_speed_cmd = 0.0f;
        uint8_t emer_rel = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET);
        uint8_t rst_prsd = (HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin) == GPIO_PIN_RESET);
        if (emer_rel && rst_prsd) {
          dev_dash.Auto.start_move  = 0;
          dev_dash.Auto.cancel_move = 0;
          g_trapezoid.is_active     = 0;
          g_scurve.is_active        = 0;
          ctrl_vel_integral         = 0.0f;
          ctrl_pos_integral         = 0.0f;
          float hold_pos = current_position * RAD_PER_CNT;
          ctrl_direct_target        = hold_pos;
          ctrl_traj_start           = hold_pos;
          dev_dash.Auto.target_deg  = hold_pos * (180.0f / 3.14159f);
          mb_slave.registers[0x24]  = 0;
          mb_slave.registers[0x01]  = 0;
          HAL_GPIO_WritePin(TOWER_R_GPIO_Port,     TOWER_R_Pin,     GPIO_PIN_RESET);
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port,   RESET_LED_Pin,   GPIO_PIN_RESET);
          if (dev_dash.Ctrl.mode == SYS_MODE_PRODUCTION) {
            GPIO_PinState mode_now = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
            current_state = (mode_now == GPIO_PIN_RESET) ? STATE_AUTO : STATE_MANUAL;
          } else {
            current_state = STATE_IDLE; // test modes → IDLE
          }
        }
      }

      // ── SET HOME: ทุกโหมด ทุก state (RT + Y หรือ set_home=1) ─────────────────
      if (!emer_active) {
        if (joy_is_connected() && joy_lt_f() > 0.5f && joy_btn(BTN_RB)) {
          dev_dash.Auto.set_home = 1;
        }
        if (dev_dash.Auto.set_home) {
          __disable_irq(); // atomic: ป้องกัน TIM7 ISR interrupt ระหว่าง reset หลายตัวแปร
          dev_dash.Auto.set_home       = 0;
          enc_reset_pending            = 1;
          ctrl_traj_start              = 0.0f;
          ctrl_direct_target           = 0.0f;
          ctrl_vel_integral            = 0.0f;
          ctrl_pos_integral            = 0.0f;
          g_trapezoid.is_active        = 0;
          g_trapezoid.p_current        = 0.0f;
          g_trapezoid.v_current        = 0.0f;
          g_scurve.is_active           = 0;
          g_scurve.p_current           = 0.0f;
          g_scurve.v_current           = 0.0f;
          motor_speed_cmd              = 0.0f;
          dev_dash.Auto.target_deg     = 0.0f;
          dev_dash.Auto.start_move     = 0;
          dev_dash.Auto.pos_err        = 0.0f;
          __enable_irq();
          sethome_led_cnt              = 1;  // trigger LED sequence (ทำหลัง enable IRQ)
        }
      }

      // 4. System Mode Selector — ข้ามเมื่อ EMER active
      if (!emer_active) {

      if (dev_dash.Ctrl.mode == SYS_MODE_HARDWARE_TEST) {
        
        // --- HARDWARE TEST UI MODE ---
        // Bypass State Machine and Modbus commands completely.
        // Apply physical outputs directly from the Live Expressions variables.
        
        HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, dev_dash.Ctrl.force_pneumatic ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, dev_dash.Ctrl.force_gripper ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, dev_dash.Ctrl.force_tower_green ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, dev_dash.Ctrl.force_tower_yellow ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, dev_dash.Ctrl.force_tower_red ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, dev_dash.Ctrl.force_emer_out ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        // Apply Forced Motor Speed
        safe_speed = dev_dash.Ctrl.force_motor_speed;
        if (safe_speed > 1.0f) safe_speed = 1.0f;
        if (safe_speed < -1.0f) safe_speed = -1.0f;
        
        if (safe_speed >= 0) {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
          safe_speed = -safe_speed;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else if (dev_dash.Ctrl.mode == SYS_MODE_JOYSTICK_TEST) {
        
        // --- JOYSTICK HARDWARE TEST MODE ---
        // Bypass State Machine. Map joystick buttons directly to physical pins for fast testing.
        if (joy_is_connected()) {
          // Face buttons = Actuators
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, joy_btn(BTN_A)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, joy_btn(BTN_B)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,   PNEUMATIC_Pin,   joy_btn(BTN_X)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GRIPPER_GPIO_Port,     GRIPPER_Pin,     joy_btn(BTN_Y)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
          // DPAD = Lights
          HAL_GPIO_WritePin(TOWER_G_GPIO_Port,   TOWER_G_Pin,   joy_btn(BTN_DPAD_UP)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_Y_GPIO_Port,   TOWER_Y_Pin,   joy_btn(BTN_DPAD_DOWN)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_R_GPIO_Port,   TOWER_R_Pin,   joy_btn(BTN_DPAD_LEFT)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, joy_btn(BTN_DPAD_RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          
          float joy_raw = (joy_rt_f() > 0.5f) ? joy_ly_f() : 0.0f;
          float joy_target = (joy_raw >  dev_dash.Ctrl.max_speed) ?  dev_dash.Ctrl.max_speed :
                             (joy_raw < -dev_dash.Ctrl.max_speed) ? -dev_dash.Ctrl.max_speed : joy_raw;
          static float joy_ramp = 0.0f;
          static uint8_t joy_dead_cnt = 0;
          float ramp_rate_local = dev_dash.Ctrl.ramp_rate;
          // Direction change dead-time: hold at 0 for 5 cycles (50ms) ก่อนสลับทิศ
          if ((joy_target > 0.0f && joy_ramp < 0.0f) ||
              (joy_target < 0.0f && joy_ramp > 0.0f)) {
            joy_dead_cnt = 5;
          }
          if (joy_dead_cnt > 0) { joy_ramp = 0.0f; joy_dead_cnt--; }
          else if (joy_target > joy_ramp + ramp_rate_local) joy_ramp += ramp_rate_local;
          else if (joy_target < joy_ramp - ramp_rate_local) joy_ramp -= ramp_rate_local;
          else                                               joy_ramp  = joy_target;
          safe_speed = joy_ramp;

          if (safe_speed >= 0) {
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
          } else {
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            safe_speed = -safe_speed;
          }
          if (safe_speed > 1.0f) safe_speed = 1.0f;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));
        } else {
          // Safe state if joystick disconnects during test
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
        }

      } else if (dev_dash.Ctrl.mode == SYS_MODE_AUTO_MOTOR_TEST) {

        // --- AUTO MOTOR TEST MODE ---
        static uint16_t auto_test_timer = 0;
        static uint8_t  auto_test_dir   = 0;
        static float    auto_ramp       = 0.0f;
        static uint8_t  auto_dead_cnt   = 0;

        uint16_t cur_period_ms = auto_test_dir ? dev_dash.Ctrl.auto_period_fwd_ms
                                                : dev_dash.Ctrl.auto_period_rev_ms;
        uint16_t half_period = (cur_period_ms / 10);
        if (half_period < 1) half_period = 1;
        auto_test_timer++;
        if (auto_test_timer >= half_period) {
          auto_test_timer = 0;
          auto_test_dir = !auto_test_dir;
          auto_dead_cnt = 5; // dead-time 50ms ก่อนสลับทิศ
        }
        float auto_target = dev_dash.Ctrl.auto_speed;
        if (auto_target > dev_dash.Ctrl.max_speed) auto_target = dev_dash.Ctrl.max_speed;
        if (auto_target < 0.0f) auto_target = 0.0f;

        if (auto_dead_cnt > 0) { auto_ramp = 0.0f; auto_dead_cnt--; }
        else if (auto_target > auto_ramp + dev_dash.Ctrl.ramp_rate) auto_ramp += dev_dash.Ctrl.ramp_rate;
        else if (auto_target < auto_ramp - dev_dash.Ctrl.ramp_rate) auto_ramp -= dev_dash.Ctrl.ramp_rate;
        else                                                          auto_ramp  = auto_target;

        safe_speed = auto_ramp;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                          auto_test_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else {

        // --- NORMAL PRODUCTION MODE ---
        // ตรวจจับ STATE_AUTO entry → init target = ตำแหน่งปัจจุบัน ป้องกันหมุนทันที
        static RobotState_t prev_state = STATE_INIT;
        if (current_state == STATE_AUTO && prev_state != STATE_AUTO) {
          float pos_now = current_position * RAD_PER_CNT;
          ctrl_direct_target    = pos_now;
          ctrl_traj_start       = pos_now;
          ctrl_vel_integral     = 0.0f;
          ctrl_pos_integral     = 0.0f;
          g_trapezoid.is_active = 0;
          g_trapezoid.p_current = 0.0f; // clear residual ป้องกัน error ตอนกลับมา AUTO
          g_trapezoid.v_current = 0.0f;
          g_scurve.is_active    = 0;
          g_scurve.p_current    = 0.0f;
          g_scurve.v_current    = 0.0f;
          if (!dev_dash.Auto.start_move) // ไม่ override ถ้ามี pending move (เช่น Go Home)
            dev_dash.Auto.target_deg = pos_now * (180.0f / 3.14159f);
          dev_dash.Auto.pos_err     = 0.0f;
        }
        prev_state = current_state;

        // ── Go Home: LT + B → วิ่งกลับ 0° ทางที่ใกล้ที่สุดด้วย PID ───────────────
        if (joy_is_connected() && joy_lt_f() > 0.5f && joy_btn(BTN_B)) {
          if (current_state == STATE_IDLE || current_state == STATE_MANUAL ||
              current_state == STATE_AUTO) {
            dev_dash.Auto.target_deg = 0.0f;
            dev_dash.Auto.start_move = 1;
            current_state = STATE_AUTO; // สลับมา AUTO ถ้ายังไม่ได้อยู่
          }
        }

        // ── Global Commands (ทำงานทุก state) ─────────────────────────────────

        // Cancel / Emergency Stop — หยุดทันที + STATE_EMER + ต้องกด RESET ถึงสั่งใหม่ได้
        if (dev_dash.Auto.cancel_move) {
          dev_dash.Auto.cancel_move = 0;
          g_trapezoid.is_active     = 0;
          g_scurve.is_active        = 0;
          ctrl_vel_integral         = 0.0f;
          ctrl_pos_integral         = 0.0f;
          motor_speed_cmd           = 0.0f;
          // Atomic: ป้องกัน ISR override PWM ระหว่าง transition
          __disable_irq();
          current_state = STATE_EMER;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          __enable_irq();
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port,   RESET_LED_Pin,   GPIO_PIN_SET);
          HAL_GPIO_WritePin(TOWER_R_GPIO_Port,     TOWER_R_Pin,     GPIO_PIN_SET);
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET);
        }

        // ── Execute State Machine ─────────────────────────────────────────────
        switch (current_state) {
          case STATE_IDLE:
            mb_slave.registers[0x27] = 0;
            if (joy_is_connected()) {
              if (joy_rt_f() > 0.5f) {
                motor_speed_cmd = joy_ly_f();              // normal speed
              } else if (joy_btn(BTN_DPAD_LEFT)) {
                motor_speed_cmd = -0.08f;                  // fine tune reverse
              } else if (joy_btn(BTN_DPAD_RIGHT)) {
                motor_speed_cmd =  0.08f;                  // fine tune forward
              } else {
                motor_speed_cmd = 0.0f;
              }
              if (joy_btn(BTN_L3) && joy_btn(BTN_R3)) current_state = STATE_EMER;
              if (joy_lt_f() > 0.5f && joy_btn(BTN_X))  current_state = STATE_CALIBRATE;
            } else {
              motor_speed_cmd = 0.0f;
            }
            // Modbus command overrides
            if (mb_slave.registers[0x01] & 0x01) { current_state = STATE_CALIBRATE; mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x02) { current_state = STATE_MANUAL; mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x04) { current_state = STATE_AUTO;   mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x10) { dev_dash.Ctrl.mode = SYS_MODE_HARDWARE_TEST; mb_slave.registers[0x01] = 0; }
            break;
            
          case STATE_CALIBRATE:
            mb_slave.registers[0x27] = 1; // Homing
            motor_speed_cmd = 0.1f; // Slow movement to find home
            if (joy_is_connected() && joy_btn(BTN_LB)) {
              current_state = STATE_EMER;
            }
            // Note: Exiting this state is handled by EXTI callback on HOME_Pin
            break;
            
          case STATE_MANUAL:
            mb_slave.registers[0x27] = 0; // Idle/Manual
            if (joy_is_connected()) {
              if (joy_btn(BTN_LB)) current_state = STATE_EMER;
              if (joy_btn(BTN_X)) HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET);
              if (joy_btn(BTN_Y)) HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
              if (joy_btn(BTN_A)) { HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET); } // Pseudo Pick
              if (joy_btn(BTN_B)) { HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET); } // Pseudo Place
              
              if (joy_rt_f() > 0.5f) {
                motor_speed_cmd = joy_ly_f(); // Deadband is applied inside joy_ly_f
              } else {
                motor_speed_cmd = 0.0f;
                if (joy_ly_f() == 0.0f) current_state = STATE_IDLE; 
              }
            } else {
              // Check Modbus Jog
              int16_t jog_cmd = (int16_t)mb_slave.registers[0x05];
              if (jog_cmd != 0) {
                // Execute jog logic here (Control team will add)
                mb_slave.registers[0x05] = 0; // Clear command
              } else if ((mb_slave.registers[0x01] & 0x02) == 0) {
                 current_state = STATE_IDLE; // Exit manual if flag cleared
              }
            }
            break;
            
          case STATE_AUTO: {
            mb_slave.registers[0x27] = 8;

            // -- Trigger: Live Expressions start_move or new Modbus target --
            static float last_mb_target = -99999.0f;
            float mb_target_deg = (float)(int16_t)mb_slave.registers[0x24];
            uint8_t do_move = 0;
            float move_deg  = 0.0f;

            if (dev_dash.Auto.start_move) {
              dev_dash.Auto.start_move = 0;
              do_move  = 1;
              move_deg = dev_dash.Auto.target_deg;
            } else if (mb_target_deg != last_mb_target) {
              last_mb_target = mb_target_deg;
              do_move  = 1;
              move_deg = mb_target_deg;
            }

            if (do_move) {
              ctrl_vel_integral  = 0.0f;
              ctrl_pos_integral  = 0.0f;
              float target_rad   = move_deg * (3.14159f / 180.0f);
              ctrl_direct_target = target_rad; // always set — ISR uses this when traj inactive

              if (dev_dash.Auto.traj_type == 2) {
                // ctrl_direct_target already set above
              } else {
                float pos_now     = current_position * RAD_PER_CNT;
                ctrl_traj_start   = pos_now;
                float disp        = target_rad - pos_now;

                g_trapezoid.v_max = dev_dash.Auto.v_max;
                g_trapezoid.a_max = dev_dash.Auto.a_max;
                g_scurve.v_max    = dev_dash.Auto.v_max;
                g_scurve.a_max    = dev_dash.Auto.a_max;
                g_scurve.j_max    = dev_dash.Auto.j_max;

                if (dev_dash.Auto.traj_type == 0) {
                  if (dev_dash.Auto.time_mode == 1)
                    Trapezoid_SetTarget_ByTime(&g_trapezoid, disp,
                      dev_dash.Auto.t_acc_seg, dev_dash.Auto.t_cruise_seg);
                  else
                    Trapezoid_SetTarget(&g_trapezoid, disp);
                } else {
                  if (dev_dash.Auto.time_mode == 1)
                    SCurve_SetTarget_ByTime(&g_scurve, disp,
                      dev_dash.Auto.t1_seg, dev_dash.Auto.t2_seg, dev_dash.Auto.t_cruise_seg);
                  else
                    SCurve_SetTarget(&g_scurve, disp);
                }
              }
            }

            // PID + trajectory update runs in TIM7 ISR @1kHz
            // Dashboard status (pos_rad, vel_rad_s, pos_err, etc.) updated there too

            // -- Abort --

            // Disabled Joystick LB Abort in AUTO to prevent floating USART3 EMI noise from triggering random EMER.
            if (mb_slave.registers[0x25] & 0x01) {
              current_state = STATE_IDLE;
              mb_slave.registers[0x25] = 0;
            }
            break;
          }
            
          case STATE_EMER:
            break; // handled by common emer_active block above
            
          default:
            current_state = STATE_IDLE;
            break;
        }
        
        // Apply Modbus Relay Control (if not controlled by Joystick)
        if (!joy_is_connected()) {
          HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, (mb_slave.registers[0x03] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, (mb_slave.registers[0x03] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, (mb_slave.registers[0x03] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, (mb_slave.registers[0x03] & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
        
        // Apply Motor Command — STATE_AUTO handles its own PWM in TIM7 ISR
        if (current_state != STATE_AUTO) {
          static float prod_ramp = 0.0f;
          static uint8_t prod_dead_cnt = 0;
          float prod_raw = motor_speed_cmd;
          if (current_state == STATE_EMER) prod_raw = 0.0f;
          float prod_target = (prod_raw >  dev_dash.Ctrl.max_speed) ?  dev_dash.Ctrl.max_speed :
                              (prod_raw < -dev_dash.Ctrl.max_speed) ? -dev_dash.Ctrl.max_speed : prod_raw;
          float prod_ramp_rate_local = dev_dash.Ctrl.ramp_rate;
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
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));
        }

      } // End Normal Production Mode
      } // End !emer_active

      // SET HOME LED feedback: เหลือง-เหลือง-เขียว (ทุกโหมด)
      if (sethome_led_cnt > 0 && !emer_active) {
        uint8_t c = sethome_led_cnt;
        if      (c <= 20)  { HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, GPIO_PIN_RESET); }
        else if (c <= 40)  { HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_RESET); }
        else if (c <= 60)  { HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_SET); }
        else if (c <= 80)  { HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_RESET); }
        else if (c <= 130) { HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_RESET); }
        else              { HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, GPIO_PIN_RESET);
                            sethome_led_cnt = 0; }
        if (sethome_led_cnt > 0) sethome_led_cnt++;
      }

      // 7. Update Live Expressions Dashboard
            dev_dash.Joy.connected = joy_is_connected();
            dev_dash.Joy.raw_buttons   = joy_raw_buttons();
            dev_dash.Joy.L_Y       = joy_ly_f();
            dev_dash.Joy.R_T       = joy_rt_f();
            dev_dash.Joy.L_T       = joy_lt_f();

            dev_dash.Status.state         = current_state;
            dev_dash.Status.motor_cmd     = (current_state == STATE_AUTO)
                                            ? motor_speed_cmd
                                            : ((HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_SET) ? safe_speed : -safe_speed);
            dev_dash.Status.encoder       = current_position;
            dev_dash.Status.current_A     = current_sensor_A;
            dev_dash.Status.pos_rad       = current_position * RAD_PER_CNT;
            { // pos_deg: 0–360° wrapped (8192 counts = 1 rev)
              int32_t pos_mod = (int32_t)current_position % 8192;
              if (pos_mod < 0) pos_mod += 8192;
              dev_dash.Status.pos_deg = (float)pos_mod * (360.0f / 8192.0f);
            }
            dev_dash.Status.vel_rad_s     = ctrl_vel_rad_s;
            dev_dash.Status.acc_rad_s2    = ctrl_acc_rad_s2;

            dev_dash.In.estop      = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin);
            dev_dash.In.mode_switch = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
            dev_dash.In.reset      = HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin);
            dev_dash.In.power      = HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin);

            dev_dash.Out.pwm       = (safe_speed > 0.0f || safe_speed < 0.0f) ? 1 : 0;
            dev_dash.Out.dir       = HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin);
            dev_dash.Out.pneumatic = HAL_GPIO_ReadPin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin);
            dev_dash.Out.gripper   = HAL_GPIO_ReadPin(GRIPPER_GPIO_Port, GRIPPER_Pin);
            dev_dash.Out.tower_g   = HAL_GPIO_ReadPin(TOWER_G_GPIO_Port, TOWER_G_Pin);
            dev_dash.Out.tower_y   = HAL_GPIO_ReadPin(TOWER_Y_GPIO_Port, TOWER_Y_Pin);
            dev_dash.Out.tower_r   = HAL_GPIO_ReadPin(TOWER_R_GPIO_Port, TOWER_R_Pin);
            dev_dash.Out.reset_led       = HAL_GPIO_ReadPin(RESET_LED_GPIO_Port, RESET_LED_Pin);
            dev_dash.Out.emer            = HAL_GPIO_ReadPin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin);
            dev_dash.Out.telemetry_active = dev_dash.Ctrl.telemetry_mode;

      // Send telemetry @ 50Hz (ทุก 2 รอบ) — 19200 baud/16 bytes ≈ 9ms ต้องการ gap ≥ 20ms
      static uint8_t telem_div = 0;
      if (++telem_div >= 2) { telem_div = 0; Send_Telemetry(); }

      // Refresh IWDG
      HAL_IWDG_Refresh(&hiwdg);
      
      // Heartbeat LED
      static uint32_t last_hb = 0;
      if (++last_hb > 50) { // 500ms at 100Hz
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        last_hb = 0;
      }
    }
  }
  /* USER CODE END 3 */
}

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
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
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
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, POWER_LATCH_Pin|PNEUMATIC_Pin|TOWER_G_Pin|TOWER_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_DIR_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GRIPPER_Pin|EMER_OUTPUT_Pin|RESET_LED_Pin|TOWER_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RESET_BTN_Pin */
  GPIO_InitStruct.Pin = RESET_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RESET_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_LATCH_Pin PNEUMATIC_Pin TOWER_G_Pin TOWER_R_Pin */
  GPIO_InitStruct.Pin = POWER_LATCH_Pin|PNEUMATIC_Pin|TOWER_G_Pin|TOWER_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_Pin LD2_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : REED_UP_Pin REED_DOWN_Pin */
  GPIO_InitStruct.Pin = REED_UP_Pin|REED_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : REED_GRIP_Pin POWER_BTN_Pin */
  GPIO_InitStruct.Pin = REED_GRIP_Pin|POWER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GRIPPER_Pin EMER_OUTPUT_Pin RESET_LED_Pin TOWER_Y_Pin */
  GPIO_InitStruct.Pin = GRIPPER_Pin|EMER_OUTPUT_Pin|RESET_LED_Pin|TOWER_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_Pin */
  GPIO_InitStruct.Pin = MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ── UART4 Init (PB9=TX AF8, PB8=RX AF8) ──────────────────────────────────────
static void UART4_Init(void)
{
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOB, &g);

  huart4.Instance          = UART4;
  huart4.Init.BaudRate     = 115200;
  huart4.Init.WordLength   = UART_WORDLENGTH_8B;
  huart4.Init.StopBits     = UART_STOPBITS_1;
  huart4.Init.Parity       = UART_PARITY_NONE;
  huart4.Init.Mode         = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);
}

// ── Telemetry Packet ─────────────────────────────────────────────────────────
// Format (16 bytes):
//   [0xAB][0xCD] [pos:f32] [vel:f32] [acc:f32] [XOR_checksum] [0x0A]
//
// Simulink: Serial Receive → 16 bytes → Unpack float × 3
//
static void Send_Telemetry(void)
{
  uint8_t buf[16];
  float pos = (float)(current_position) * RAD_PER_CNT;
  float vel = (float)ctrl_vel_rad_s;
  float acc = (float)ctrl_acc_rad_s2;

  buf[0]  = 0x7E;
  buf[1]  = 0x7E;
  memcpy(&buf[2],  &pos, 4);
  memcpy(&buf[6],  &vel, 4);
  memcpy(&buf[10], &acc, 4);
  buf[14] = 0x03;
  buf[15] = 0x03;

  // Single-Cable Multiplexed Mode: Always send via ST-Link (LPUART1)
  // Use a very short timeout (2ms) so it doesn't block the loop if Modbus is busy
  HAL_UART_Transmit(&hlpuart1, buf, sizeof(buf), 2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // ESTOP is now safely polled in the 100Hz loop to prevent EMI phantom triggers and ISR hangs.
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
    // 1kHz Loop
    // Delta accumulation — handles 16-bit wraparound correctly
    static uint16_t enc_last = 0;
    uint16_t enc_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    if (enc_reset_pending) {
      enc_reset_pending = 0;
      enc_last         = enc_now; // sync enc_last กับ counter ปัจจุบัน
      current_position = 0;       // delta ถัดไปจะเป็น 0 พอดี
    }

    int32_t enc_delta = -(int32_t)(int16_t)(enc_now - enc_last);
    enc_last = enc_now;
    current_position += enc_delta;

    // Windowed velocity (VEL_WIN-sample ring buffer @ 1kHz)
    static int32_t vel_buf[VEL_WIN];
    static uint8_t vel_idx = 0;
    vel_buf[vel_idx] = enc_delta;
    vel_idx = (vel_idx + 1) % VEL_WIN;
    int32_t vel_sum = 0;
    for (int vi = 0; vi < VEL_WIN; vi++) vel_sum += vel_buf[vi];
    // vel = (sum/WIN counts/tick) * (2π/8192 rad/count) / (0.001 s/tick)
    ctrl_vel_rad_s = (float)vel_sum * (6.28318f / (8192.0f * VEL_WIN * 0.001f));

    // Acceleration = dv/dt @ 1kHz, LPF alpha ปรับได้ใน dev_dash.Ctrl.acc_alpha
    static float prev_vel_for_acc = 0.0f;
    float raw_acc = (ctrl_vel_rad_s - prev_vel_for_acc) * 1000.0f;
    prev_vel_for_acc = ctrl_vel_rad_s;
    float alpha = dev_dash.Ctrl.acc_alpha;
    if (alpha < 0.01f) alpha = 0.01f;
    if (alpha > 1.0f)  alpha = 1.0f;
    ctrl_acc_rad_s2 = ctrl_acc_rad_s2 * (1.0f - alpha) + raw_acc * alpha;

    // ── AUTO CONTROL @ 1kHz ──────────────────────────────────────────────────
    if (current_state == STATE_AUTO && dev_dash.Ctrl.mode == SYS_MODE_PRODUCTION) {
      // Trajectory reference
      float ideal_pos, ideal_vel;
      if (dev_dash.Auto.traj_type == 2) {
        ideal_pos = ctrl_direct_target;
        ideal_vel = 0.0f;
      } else if (dev_dash.Auto.traj_type == 0) {
        Trapezoid_Update(&g_trapezoid);
        if (g_trapezoid.is_active) {
          ideal_pos = ctrl_traj_start + g_trapezoid.p_current;
          ideal_vel = g_trapezoid.v_current;
        } else {
          ideal_pos = ctrl_direct_target; // trajectory done → hold target, p_current residual ignored
          ideal_vel = 0.0f;
        }
      } else {
        SCurve_Update(&g_scurve);
        if (g_scurve.is_active) {
          ideal_pos = ctrl_traj_start + g_scurve.p_current;
          ideal_vel = g_scurve.v_current;
        } else {
          ideal_pos = ctrl_direct_target; // trajectory done → hold target
          ideal_vel = 0.0f;
        }
      }

      float pos_rad = current_position * RAD_PER_CNT;
      float pos_err = ideal_pos - pos_rad;

      // Position PID (outer loop) → velocity setpoint
      ctrl_pos_integral += pos_err * 0.001f;
      if (dev_dash.Auto.ki_pos > 1e-6f) {
        float lim = dev_dash.Auto.v_max / dev_dash.Auto.ki_pos;
        if (ctrl_pos_integral >  lim) ctrl_pos_integral =  lim;
        if (ctrl_pos_integral < -lim) ctrl_pos_integral = -lim;
      }
      float vel_sp = dev_dash.Auto.kp_pos * pos_err
                   + dev_dash.Auto.ki_pos  * ctrl_pos_integral
                   + ideal_vel;

      // Velocity PID (inner loop) → PWM%
      float vel_err = vel_sp - ctrl_vel_rad_s;
      static float prev_vel_err_isr = 0.0f;
      float d_vel = (vel_err - prev_vel_err_isr) * 1000.0f;
      prev_vel_err_isr = vel_err;
      // Anti-windup: integrate เฉพาะเมื่อ output ไม่ saturate (ป้องกันหมุนไม่หยุดตอน trajectory ยาว)
      float pwm_pct = dev_dash.Auto.kp_vel * vel_err
                    + dev_dash.Auto.ki_vel  * ctrl_vel_integral
                    + dev_dash.Auto.kd_vel  * d_vel;
      uint8_t vel_sat = (pwm_pct >= 95.0f && vel_err > 0.0f) ||
                        (pwm_pct <= -95.0f && vel_err < 0.0f);
      if (!vel_sat) ctrl_vel_integral += vel_err * 0.001f;
      if (dev_dash.Auto.ki_vel > 1e-6f) {
        float lim = 20.0f / dev_dash.Auto.ki_vel; // clamp ไม่เกิน 20% ของ PWM range
        if (ctrl_vel_integral >  lim) ctrl_vel_integral =  lim;
        if (ctrl_vel_integral < -lim) ctrl_vel_integral = -lim;
      }
      pwm_pct = dev_dash.Auto.kp_vel * vel_err  // คำนวณใหม่หลัง integral update
              + dev_dash.Auto.ki_vel  * ctrl_vel_integral
              + dev_dash.Auto.kd_vel  * d_vel;

      // Invert motor direction ถ้า error เพิ่มแทนที่จะลด
      if (dev_dash.Auto.motor_invert) pwm_pct = -pwm_pct;

      if (pwm_pct >  100.0f) pwm_pct =  100.0f;
      if (pwm_pct < -100.0f) pwm_pct = -100.0f;

      // Hard stop when settled
      uint8_t traj_done = (dev_dash.Auto.traj_type == 0) ? (!g_trapezoid.is_active) :
                          (dev_dash.Auto.traj_type == 1) ? (!g_scurve.is_active)    : 1;
      if (traj_done && fabsf(pos_err) < 0.035f && fabsf(ctrl_vel_rad_s) < 0.1f) {
        pwm_pct           = 0.0f;
        ctrl_vel_integral = 0.0f;
        ctrl_pos_integral = 0.0f;
      }

      // Apply motor directly (ISR bypasses slew — PID already smooth)
      // Dead-zone 2%: ถ้า |spd| < 0.02 ไม่สลับ DIR (ป้องกัน rapid flip ที่ 1kHz → fighting)
      float spd = pwm_pct / 100.0f;
      if (spd >  dev_dash.Ctrl.max_speed) spd =  dev_dash.Ctrl.max_speed;
      if (spd < -dev_dash.Ctrl.max_speed) spd = -dev_dash.Ctrl.max_speed;
      if (spd > 0.02f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
      } else if (spd < -0.02f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        spd = -spd;
      } else {
        spd = 0.0f; // dead-zone — ไม่สลับ DIR
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                            (uint32_t)(spd * (float)htim3.Init.Period));

      // Update shared status (read by main loop for dashboard)
      motor_speed_cmd             = pwm_pct / 100.0f;
      dev_dash.Auto.pos_rad       = pos_rad;
      dev_dash.Auto.vel_rad_s     = ctrl_vel_rad_s;
      dev_dash.Auto.pos_ideal     = ideal_pos;
      dev_dash.Auto.vel_ideal     = ideal_vel;
      dev_dash.Auto.pos_err       = pos_err;
      dev_dash.Auto.vel_sp        = vel_sp;
      dev_dash.Auto.pwm_out       = motor_speed_cmd;
      dev_dash.Auto.traj_active   = (dev_dash.Auto.traj_type == 0) ? g_trapezoid.is_active :
                                    (dev_dash.Auto.traj_type == 1) ? g_scurve.is_active    :
                                    (fabsf(pos_err) > 0.035f) ? 1 : 0;
    }
    // ── END AUTO CONTROL ─────────────────────────────────────────────────────
    // Safety: ถ้าเพิ่งออกจาก STATE_AUTO → force PWM=0 ทันทีใน ISR tick ถัดไป
    {
      static uint8_t isr_was_auto = 0;
      if (current_state == STATE_AUTO) {
        isr_was_auto = 1;
      } else if (isr_was_auto) {
        isr_was_auto = 0;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // force PWM=0
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
