#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>

/* =========================================================================
 * PID Control — variables, gains, debug observables
 * Mirrors the control team's Auto_Control structure.
 * Include in main.c only.
 * =========================================================================*/

/* ── Master enables ──────────────────────────────────────────────────────── */
extern volatile uint8_t pid_enabled;       // 0 = freeze all PIDs + hold PWM=0
extern volatile uint8_t hard_stop_active;  // 1 = force PWM=0 this tick (ISR-cleared)

/* ── Velocity PID gains (Live Expressions) ───────────────────────────────── */
extern float kp_vel;
extern float ki_vel;
extern float kd_vel;

/* ── Position PID gains (Live Expressions) ───────────────────────────────── */
extern float kp_pos;
extern float ki_pos;
extern float kd_pos;

/* ── Integrator state ────────────────────────────────────────────────────── */
extern float vel_integral;
extern float pos_integral;
extern float max_pwm;

/* ── Debug observables (read-only in Live Expressions) ──────────────────── */
extern volatile float vel_error_live;  // current velocity error (rad/s)
extern volatile float i_term_live;     // current vel integral accumulator
extern volatile float ss_error_deg;    // steady-state position error (deg)
extern volatile float ss_error_rad;    // steady-state position error (rad)

/* ── Functions ───────────────────────────────────────────────────────────── */
float Velocity_PID(float error, float dt);
float Position_PID(float pos_error, float vel_feedback, float dt);
void  Velocity_PID_Reset(void);
void  Position_PID_Reset(void);

#endif /* PID_CONTROL_H */
