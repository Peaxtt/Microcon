#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "main.h"
#include "robot.h"
#include "ENCODER.h"

/* =========================================================================
 * hardware.h — Hardware Abstraction Layer
 *
 * Wraps all direct hardware access: GPIO reads/writes, ADC, PWM, encoder.
 * Higher layers (state_machine, modbus_app) never touch HAL directly.
 * ========================================================================= */

/* HOME_SENSOR pin polarity: PC3, PULLUP, active when HIGH */
#define HOME_SENSOR_ACTIVE  GPIO_PIN_SET

/* ── Startup initialisation ──────────────────────────────────────────────── */

/*
 * Call once in USER CODE 2 after HAL init and CubeMX-generated inits.
 *   htim_pwm : TIM3 (PWM Channel 4, PC9)
 *   htim_enc : TIM1 (quadrature encoder PA8/PA9)
 *   hadc_cur : ADC2 (current sensor DMA, PA0)
 *   adc_buf  : pointer to DMA buffer (40 samples)
 */
void hardware_init(TIM_HandleTypeDef *htim_pwm,
                   TIM_HandleTypeDef *htim_enc,
                   ADC_HandleTypeDef *hadc_cur,
                   uint32_t          *adc_buf);

/* ── 100 Hz sensor poll ──────────────────────────────────────────────────── */

/*
 * Read all sensors and write results to robot.h globals.
 * Call every 10ms in the main loop (after modbus_process, before state_machine).
 *
 * Updates:
 *   robot_estop, robot_reed_*, robot_home_sensor
 *   robot_reset_btn, robot_power_btn, robot_mode_sw
 *   robot_current_A, robot_pos_deg (display, from robot_pos_rad)
 */
void hardware_sensors_update(void);

/* ── Output drivers ──────────────────────────────────────────────────────── */

/* Tower light: pass 0 or 1 for each colour */
void hardware_set_tower(uint8_t red, uint8_t yellow, uint8_t green);

/* Motor PWM: -1.0 = full reverse, 0 = stop, +1.0 = full forward
 * Respects sys_max_speed and dead-zone (min_pwm_pct).
 * Sets TIM3 CH4 compare value and MOTOR_DIR pin. */
void hardware_set_motor(float fraction);

/* Pneumatic cylinder and gripper actuators */
void hardware_set_actuator(uint8_t cylinder, uint8_t grip_open, uint8_t grip_close);

/* Emergency output relay */
void hardware_set_emer_output(uint8_t active);

/* Power latch (keeps board powered after button release) */
void hardware_set_power_latch(uint8_t on);

/* RESET button LED */
void hardware_set_reset_led(uint8_t on);

/* ── Homing sensor EXTI ──────────────────────────────────────────────────── */

/* Enable rising-edge interrupt on PC3 (HOME_SENSOR).
 * Call at start of each homing phase that needs sensor detection. */
void hardware_homing_sensor_enable(void);

/* Disable interrupt and clear pending flag. */
void hardware_homing_sensor_disable(void);

/* ── Homing completion ───────────────────────────────────────────────────── */

/*
 * Call from state_machine when the slow homing sensor triggers.
 * Actions (atomic, IRQ disabled):
 *   - Disables homing EXTI
 *   - Sets encoder_reset_req = 1  (ISR zeroes encoder next tick)
 *   - Zeroes robot_cumul_deg
 *   - Stops motor: robot_motor_pct = 0
 *   - Sets robot_homed = 1
 *   - If home_offset_deg != 0: arms a move to home_offset_deg
 */
void hardware_finish_homing(void);

/* ── Encoder access ──────────────────────────────────────────────────────── */

/* Encoder instance (used by ISR directly) */
extern Encoder_t encoder;

/* Minimum PWM fraction to overcome motor static friction */
extern volatile float min_pwm_fraction;   // default 0.05

#endif /* __HARDWARE_H */
