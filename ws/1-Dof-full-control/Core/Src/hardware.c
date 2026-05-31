#include "hardware.h"
#include <math.h>

/* ── Private handles (set by hardware_init) ─────────────────────────────── */
static TIM_HandleTypeDef *pwm_tim   = NULL;
static TIM_HandleTypeDef *enc_tim   = NULL;
static ADC_HandleTypeDef *cur_adc   = NULL;
static uint32_t          *adc_buf   = NULL;

Encoder_t encoder;
volatile float min_pwm_fraction = 0.05f;

/* ADC calibration (Live Expressions tunable) */
volatile float cur_zero_volts   = 2.46f;  // voltage at zero current
volatile float cur_sensitivity  = 0.066f; // V/A (ACS712 30A)

/* Reed switch debounce timer (1ms per count) */
static volatile uint32_t reed_dummy_timer = 0;

/* ── hardware_init ──────────────────────────────────────────────────────── */
void hardware_init(TIM_HandleTypeDef *htim_pwm,
                   TIM_HandleTypeDef *htim_enc,
                   ADC_HandleTypeDef *hadc_cur,
                   uint32_t          *dma_buf)
{
  pwm_tim = htim_pwm;
  enc_tim = htim_enc;
  cur_adc = hadc_cur;
  adc_buf = dma_buf;
  Encoder_Init(&encoder, 0.001f);
}

/* ── hardware_sensors_update (100Hz) ─────────────────────────────────────── */
void hardware_sensors_update(void)
{
  /* ESTOP: PA15 PULLUP, active LOW */
  robot_estop = (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) ? 1 : 0;

  /* Reed switches */
  if (reed_dummy_en) {
    /* Dummy mode: simulate reed based on motor position (for bench testing) */
    /* Real values stay at 0 — state_machine uses its own timing logic */
    robot_reed_up   = 0;
    robot_reed_down = 0;
    robot_reed_grip = 0;
  } else {
    robot_reed_up   = (HAL_GPIO_ReadPin(REED_UP_GPIO_Port,   REED_UP_Pin)   == GPIO_PIN_SET) ? 1 : 0;
    robot_reed_down = (HAL_GPIO_ReadPin(REED_DOWN_GPIO_Port, REED_DOWN_Pin) == GPIO_PIN_SET) ? 1 : 0;
    robot_reed_grip = (HAL_GPIO_ReadPin(REED_GRIP_GPIO_Port, REED_GRIP_Pin) == GPIO_PIN_SET) ? 1 : 0;
  }

  /* Home sensor: PC3, active HIGH */
  robot_home_sensor = (HAL_GPIO_ReadPin(HOME_SENSOR_GPIO_Port, HOME_SENSOR_Pin) == HOME_SENSOR_ACTIVE) ? 1 : 0;

  /* Buttons (all PULLUP, active LOW) */
  robot_reset_btn = (HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin) == GPIO_PIN_RESET) ? 1 : 0;
  robot_power_btn = (HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin) == GPIO_PIN_RESET) ? 1 : 0;

  /* Cabinet mode switch: PC2, HIGH = MANUAL, LOW = AUTO */
  robot_mode_sw = (HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin) == GPIO_PIN_SET) ? 1 : 0;

  /* Current sensor: ADC DMA average */
  if (adc_buf != NULL) {
    uint32_t sum = 0;
    for (int i = 0; i < 40; i++) sum += adc_buf[i];
    float adc_v = ((float)(sum / 40u) / 4095.0f) * 3.3f;
    robot_current_A = (adc_v - cur_zero_volts) / cur_sensitivity;
  }

  /* Display position: wrap to 0–359.99° */
  float pd = robot_pos_rad * (180.0f / 3.14159265f);
  pd = fmodf(pd, 360.0f);
  if (pd < 0.0f) pd += 360.0f;
  robot_pos_deg = pd;
}

/* ── hardware_set_tower ─────────────────────────────────────────────────── */
void hardware_set_tower(uint8_t red, uint8_t yellow, uint8_t green)
{
  HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, red    ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, yellow ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, green  ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── hardware_set_motor ─────────────────────────────────────────────────── */
void hardware_set_motor(float fraction)
{
  /* Clamp to max speed */
  float limit = sys_max_speed;
  if (fraction >  limit) fraction =  limit;
  if (fraction < -limit) fraction = -limit;

  /* Dead zone — apply min threshold if non-zero command */
  if (fraction > 0.01f) {
    if (fraction < min_pwm_fraction) fraction = min_pwm_fraction;
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
  } else if (fraction < -0.01f) {
    fraction = -fraction;
    if (fraction < min_pwm_fraction) fraction = min_pwm_fraction;
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
  } else {
    fraction = 0.0f;
  }

  if (pwm_tim != NULL) {
    uint32_t cnt = (uint32_t)(fraction * (float)pwm_tim->Init.Period);
    __HAL_TIM_SET_COMPARE(pwm_tim, TIM_CHANNEL_4, cnt);
  }
}

/* ── hardware_set_actuator ──────────────────────────────────────────────── */
void hardware_set_actuator(uint8_t cylinder, uint8_t grip_open, uint8_t grip_close)
{
  HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin,
                    cylinder   ? GPIO_PIN_SET : GPIO_PIN_RESET);
  /* GRIPPER pin: LOW=close, HIGH=open (active-high open) */
  if (grip_open && !grip_close)
    HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET);
  else if (grip_close && !grip_open)
    HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
}

/* ── hardware_set_emer_output ───────────────────────────────────────────── */
void hardware_set_emer_output(uint8_t active)
{
  HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin,
                    active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── hardware_set_power_latch ───────────────────────────────────────────── */
void hardware_set_power_latch(uint8_t on)
{
  HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── hardware_set_reset_led ─────────────────────────────────────────────── */
void hardware_set_reset_led(uint8_t on)
{
  HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── hardware_homing_sensor_enable ─────────────────────────────────────── */
void hardware_homing_sensor_enable(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(HOME_SENSOR_Pin);
  robot_home_sensor = 0;
  GPIO_InitTypeDef g = {0};
  g.Pin   = HOME_SENSOR_Pin;
  g.Mode  = GPIO_MODE_IT_RISING;
  g.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_SENSOR_GPIO_Port, &g);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* ── hardware_homing_sensor_disable ────────────────────────────────────── */
void hardware_homing_sensor_disable(void)
{
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(HOME_SENSOR_Pin);
  GPIO_InitTypeDef g = {0};
  g.Pin  = HOME_SENSOR_Pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_SENSOR_GPIO_Port, &g);
}

/* ── hardware_finish_homing ─────────────────────────────────────────────── */
void hardware_finish_homing(void)
{
  hardware_homing_sensor_disable();
  __disable_irq();
  encoder_reset_req  = 1;
  robot_cumul_deg    = 0.0f;
  robot_motor_pct    = 0.0f;
  __enable_irq();
  hardware_set_motor(0.0f);
  robot_homed = 1;

  /* Arm home offset move if needed */
  if (home_offset_deg != 0.0f) {
    robot_target_rad  = home_offset_deg * (3.14159265f / 180.0f);
    robot_move_armed  = 1;
  }
}
