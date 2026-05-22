#include "pid_control.h"

/* ── Master enables ──────────────────────────────────────────────────────── */
volatile uint8_t pid_enabled      = 1;   // default ON — set 0 in Live Expressions to freeze
volatile uint8_t hard_stop_active = 0;   // set 1 to force PWM=0 one tick, then auto-clears

/* ── Velocity PID gains ───────────────────────────────────────────────────── */
float kp_vel = 30.0f;
float ki_vel =  0.1f;
float kd_vel =  0.0f;

/* ── Position PID gains ──────────────────────────────────────────────────── */
float kp_pos = 1.0f;
float ki_pos = 0.0f;
float kd_pos = 0.0f;

/* ── Integrator state ────────────────────────────────────────────────────── */
float vel_integral = 0.0f;
float pos_integral = 0.0f;
float max_pwm      = 100.0f;

/* ── Debug observables ───────────────────────────────────────────────────── */
volatile float vel_error_live = 0.0f;
volatile float i_term_live    = 0.0f;
volatile float ss_error_deg   = 0.0f;
volatile float ss_error_rad   = 0.0f;

/* =========================================================================
 * Velocity_PID
 *   error : vel_setpoint − vel_feedback  (rad/s)
 *   dt    : sample period                (s)
 *   returns: PWM duty (%)  −100..+100
 * ========================================================================= */
float Velocity_PID(float error, float dt)
{
    float p_out = kp_vel * error;

    vel_integral += error * dt;
    float max_int = (ki_vel > 0.0f) ? (max_pwm / ki_vel) : 0.0f;
    if (vel_integral >  max_int) vel_integral =  max_int;
    if (vel_integral < -max_int) vel_integral = -max_int;
    float i_out = ki_vel * vel_integral;

    float d_out = kd_vel * ((error - vel_error_live) / dt);  // d(error)/dt

    vel_error_live = error;
    i_term_live    = vel_integral;

    float out = p_out + i_out + d_out;
    if (out >  max_pwm) out =  max_pwm;
    if (out < -max_pwm) out = -max_pwm;
    return out;
}

/* =========================================================================
 * Position_PID
 *   pos_error    : ideal_pos − actual_pos  (rad)
 *   vel_feedback : filtered actual velocity (rad/s) — used for D-term
 *   dt           : sample period            (s)
 *   returns: velocity setpoint (rad/s) for cascade mode
 * ========================================================================= */
float Position_PID(float pos_error, float vel_feedback, float dt)
{
    float p_out = kp_pos * pos_error;

    pos_integral += pos_error * dt;
    float max_pos_int = (ki_pos > 0.0f) ? (10.0f / ki_pos) : 0.0f;
    if (pos_integral >  max_pos_int) pos_integral =  max_pos_int;
    if (pos_integral < -max_pos_int) pos_integral = -max_pos_int;
    float i_out = ki_pos * pos_integral;

    float d_out = -kd_pos * vel_feedback;  // negate velocity = d(pos_error)/dt

    ss_error_rad = pos_error;
    ss_error_deg = pos_error * (180.0f / 3.14159265f);

    return p_out + i_out + d_out;
}

void Velocity_PID_Reset(void)
{
    vel_integral   = 0.0f;
    vel_error_live = 0.0f;
    i_term_live    = 0.0f;
}

void Position_PID_Reset(void)
{
    pos_integral = 0.0f;
    ss_error_rad = 0.0f;
    ss_error_deg = 0.0f;
}
