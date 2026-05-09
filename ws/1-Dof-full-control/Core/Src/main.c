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
#include "joystick.h"
#include "modbus.h"
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
float motor_speed_cmd = 0.0f; // -1.0 to 1.0
int32_t current_position = 0;
uint16_t current_sensor_val = 0;

// Power Latch Timer
uint32_t power_btn_hold_ms = 0;

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
  float    auto_speed;     // speed 0.0-1.0
  uint16_t auto_period_ms; // ms ต่อทิศ
  // Current sensor calibration (WCS1800)
  float    cur_zero_v;     // voltage ที่ 0A (วัดจาก multimeter ตอนไม่มีกระแส)
  float    cur_sens;       // sensitivity V/A (0.066 = 66mV/A)
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
  uint16_t current_mA;
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
} DashOut_t;

typedef struct {
  DashCtrl_t   Ctrl;
  DashJoy_t    Joy;
  DashStatus_t Status;
  DashIn_t     In;
  DashOut_t    Out;
} DevDashboard_t;

DevDashboard_t dev_dash = {
  .Ctrl.mode          = SYS_MODE_PRODUCTION,
  .Ctrl.ramp_rate     = 0.03f,
  .Ctrl.max_speed     = 0.40f,
  .Ctrl.auto_speed    = 0.30f,
  .Ctrl.auto_period_ms= 1000,
  .Ctrl.cur_zero_v    = 1.65f,  // ← วัด multimeter ที่ PC5 ตอน 0A แล้วใส่ค่าจริง
  .Ctrl.cur_sens      = 0.066f, // 66mV/A สำหรับ WCS1800
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (flag_10ms) {
      flag_10ms = 0;
      
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
          current_sensor_val = (uint16_t)(i_a * 1000.0f); // mA
        }
      }

      // 2. Process incoming Modbus frames
      modbus_process(&mb_slave);

      // 3. Update Status Registers (0x26 - 0x31)
      mb_slave.registers[0x26] = (HAL_GPIO_ReadPin(REED_UP_GPIO_Port, REED_UP_Pin) == GPIO_PIN_RESET ? 1 : 0) |
                                 (HAL_GPIO_ReadPin(REED_DOWN_GPIO_Port, REED_DOWN_Pin) == GPIO_PIN_RESET ? 2 : 0) |
                                 (HAL_GPIO_ReadPin(REED_GRIP_GPIO_Port, REED_GRIP_Pin) == GPIO_PIN_RESET ? 4 : 0);
      
      mb_slave.registers[0x28] = (uint16_t)current_position; 
      mb_slave.registers[0x31] = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;
      mb_slave.registers[0x04] = current_sensor_val; // Save filtered ADC to Modbus
      
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

      // Handle Hardware Selector Switch (Auto/Manual on MODE_Pin)
      static GPIO_PinState last_mode_switch = GPIO_PIN_RESET;
      GPIO_PinState current_mode_switch = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
      
      if (current_mode_switch != last_mode_switch) {
        if (current_state != STATE_EMER && dev_dash.Ctrl.mode == SYS_MODE_PRODUCTION) {
          // Default assumption: SET (High) = AUTO, RESET (Low) = MANUAL.
          current_state = (current_mode_switch == GPIO_PIN_SET) ? STATE_AUTO : STATE_MANUAL;
        }
        last_mode_switch = current_mode_switch;
      }

      // 4. System Mode Selector (Production vs Hardware Test UI)
      float safe_speed = 0.0f;

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
          HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, joy_btn(BTN_Y) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, joy_btn(BTN_B) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, joy_btn(BTN_X) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          
          HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, joy_btn(BTN_DPAD_UP) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, joy_btn(BTN_DPAD_DOWN) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, joy_btn(BTN_DPAD_LEFT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, joy_btn(BTN_DPAD_RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
          
          float joy_raw = (joy_rt_f() > 0.5f) ? joy_ly_f() : 0.0f;
          float joy_target = (joy_raw >  dev_dash.Ctrl.max_speed) ?  dev_dash.Ctrl.max_speed :
                             (joy_raw < -dev_dash.Ctrl.max_speed) ? -dev_dash.Ctrl.max_speed : joy_raw;
          static float joy_ramp = 0.0f;
          float ramp_rate_local = dev_dash.Ctrl.ramp_rate;
          if      (joy_target > joy_ramp + ramp_rate_local) joy_ramp += ramp_rate_local;
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
        uint16_t half_period = (dev_dash.Ctrl.auto_period_ms / 10);
        if (half_period < 1) half_period = 1;
        auto_test_timer++;
        if (auto_test_timer >= half_period) {
          auto_test_timer = 0;
          auto_test_dir = !auto_test_dir;
        }
        safe_speed = dev_dash.Ctrl.auto_speed;
        if (safe_speed > dev_dash.Ctrl.max_speed) safe_speed = dev_dash.Ctrl.max_speed;
        if (safe_speed < 0.0f) safe_speed = 0.0f;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                          auto_test_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } else {

        // --- NORMAL PRODUCTION MODE ---
        // Execute State Machine
        switch (current_state) {
          case STATE_IDLE:
            mb_slave.registers[0x27] = 0; // Idle
            motor_speed_cmd = 0.0f;
            
            if (joy_is_connected() && joy_btn(BTN_L3) && joy_btn(BTN_R3)) {
              current_state = STATE_EMER;
            } else if (joy_is_connected() && joy_lt_f() > 0.5f && joy_btn(BTN_X)) {
              current_state = STATE_CALIBRATE;
            } else if (joy_is_connected() && joy_rt_f() > 0.5f && (joy_ly_f() > 0.1f || joy_ly_f() < -0.1f)) {
              current_state = STATE_MANUAL;
            }
            
            // Modbus command overrides
            if (mb_slave.registers[0x01] & 0x01) { current_state = STATE_CALIBRATE; mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x02) { current_state = STATE_MANUAL; mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x04) { current_state = STATE_AUTO; mb_slave.registers[0x01] = 0; }
            else if (mb_slave.registers[0x01] & 0x10) { dev_dash.Ctrl.mode = SYS_MODE_HARDWARE_TEST; mb_slave.registers[0x01] = 0; } // Enter Test Mode via Modbus
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
            
          case STATE_AUTO:
            mb_slave.registers[0x27] = 8; // P2P Task running
            if (joy_is_connected() && joy_btn(BTN_LB)) current_state = STATE_EMER;
            if (mb_slave.registers[0x25] & 0x01) {
              current_state = STATE_IDLE; // Soft stop
              mb_slave.registers[0x25] = 0;
            }
            break;
            
          case STATE_EMER:
            mb_slave.registers[0x27] = 0;
            motor_speed_cmd = 0.0f;
            HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, GPIO_PIN_SET); // Red Tower Light on PC0
            HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET); // EMER_OUTPUT on PB6
            
            if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET) {
              if ((joy_is_connected() && joy_btn(BTN_BACK)) || (mb_slave.registers[0x01] == 0xFF)) {
                current_state = STATE_IDLE;
                mb_slave.registers[0x01] = 0;
                HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET); 
              }
            }
            break;
            
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
        
        // Apply Motor Command (To Cytron) with slew rate limiter
        float prod_raw = motor_speed_cmd;
        if (current_state == STATE_EMER || current_state == STATE_IDLE) prod_raw = 0.0f;
        float prod_target = (prod_raw >  dev_dash.Ctrl.max_speed) ?  dev_dash.Ctrl.max_speed :
                            (prod_raw < -dev_dash.Ctrl.max_speed) ? -dev_dash.Ctrl.max_speed : prod_raw;
        static float prod_ramp = 0.0f;
        float prod_ramp_rate_local = dev_dash.Ctrl.ramp_rate;
        if      (prod_target > prod_ramp + prod_ramp_rate_local) prod_ramp += prod_ramp_rate_local;
        else if (prod_target < prod_ramp - prod_ramp_rate_local) prod_ramp -= prod_ramp_rate_local;
        else                                                      prod_ramp  = prod_target;
        safe_speed = prod_ramp;

        if (safe_speed >= 0) {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
          safe_speed = -safe_speed;
        }
        if (safe_speed > 1.0f) safe_speed = 1.0f;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(safe_speed * (float)htim3.Init.Period));

      } // End Normal Production Mode

      // 7. Update Live Expressions Dashboard
            dev_dash.Joy.connected = joy_is_connected();
            dev_dash.Joy.raw_buttons   = joy_raw_buttons();
            dev_dash.Joy.L_Y       = joy_ly_f();
            dev_dash.Joy.R_T       = joy_rt_f();
            dev_dash.Joy.L_T       = joy_lt_f();

            dev_dash.Status.state         = current_state;
            // Reconstruct actual signed motor command for dashboard
            dev_dash.Status.motor_cmd     = (HAL_GPIO_ReadPin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin) == GPIO_PIN_SET) ? safe_speed : -safe_speed;
            dev_dash.Status.encoder       = current_position;
            dev_dash.Status.current_mA    = current_sensor_val;

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
            dev_dash.Out.reset_led = HAL_GPIO_ReadPin(RESET_LED_GPIO_Port, RESET_LED_Pin);
            dev_dash.Out.emer      = HAL_GPIO_ReadPin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin);

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
  sConfig.Channel = ADC_CHANNEL_11;
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
  hlpuart1.Init.BaudRate = 19200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
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
  HAL_GPIO_WritePin(GPIOA, MOTOR_DIR_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GRIPPER_Pin|POWER_LATCH_Pin|RESET_LED_Pin|EMER_OUTPUT_Pin
                          |TOWER_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PNEUMATIC_Pin|TOWER_G_Pin|TOWER_R_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : REED_GRIP_Pin RESET_BTN_Pin POWER_BTN_Pin */
  GPIO_InitStruct.Pin = REED_GRIP_Pin|RESET_BTN_Pin|POWER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GRIPPER_Pin POWER_LATCH_Pin RESET_LED_Pin EMER_OUTPUT_Pin
                           TOWER_Y_Pin */
  GPIO_InitStruct.Pin = GRIPPER_Pin|POWER_LATCH_Pin|RESET_LED_Pin|EMER_OUTPUT_Pin
                          |TOWER_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PNEUMATIC_Pin */
  GPIO_InitStruct.Pin = PNEUMATIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PNEUMATIC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOWER_G_Pin TOWER_R_Pin */
  GPIO_InitStruct.Pin = TOWER_G_Pin|TOWER_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_Pin */
  GPIO_InitStruct.Pin = MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_Pin) {
    // E-Stop Triggered (Falling Edge)
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) {
      current_state = STATE_EMER;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // Force PWM to 0 immediately
      HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET); // Set EMER_OUTPUT
    }
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
    // 1kHz Loop
    current_position = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim1); // int16_t cast handles signed 16-bit wrap
    
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
