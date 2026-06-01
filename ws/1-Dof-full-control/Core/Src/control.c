/*
 * control.c — Due's cascade PID algorithm (ported to our interface)
 *
 * Source: Control_by_due / Auto_Control / Core/Src/main.c
 * Algorithm:
 *   TIM7 (500 Hz, outer) : trajectory advance → position PID → vel_command
 *   TIM6 (1 kHz, inner)  : velocity PID + RefFF → PWM
 *
 * In our firmware both loops are called from the same 1 kHz ISR:
 *   - velocity PID : every call   (1 kHz)
 *   - position PID : every 2nd call (500 Hz effective)
 *
 * Interface: control_init / control_reset / control_set_target /
 *            control_update / control_is_settled
 *
 * Gains and limits are read from control_model[0] every call so
 * they can be updated via dev_dash.Ctrl without reflashing.
 */

#include "control.h"
#include "SCURVE.h"
#include "TRAPEZOID.h"
#include "REF_FEEDFORWARD.h"
#include "DistFF.h"
#include <math.h>
#include <string.h>

/* ── Timing ───────────────────────────────────────────────────────────────── */
#define CTRL_DT      0.001f   /* 1 kHz — velocity loop dt */
#define CTRL_POS_DT  0.002f   /* 500 Hz — position loop dt (every 2nd tick) */

/* ── Trajectory generators (Due uses one of these at a time) ─────────────── */
static SCurve_t     g_sc;
static Trapezoid_t  g_tr;
static RefFF_t      g_rff;
static DistFF_t     g_dff;

static float  traj_start_pos  = 0.0f;
static float  traj_target     = 0.0f;  /* in radians */

/* ── PID state (Due's variables) ─────────────────────────────────────────── */
static float vel_integral    = 0.0f;
static float vel_prev_error  = 0.0f;
static float pos_integral    = 0.0f;

/* ── Inter-loop interface: pos loop (500Hz) writes, vel loop (1kHz) reads ── */
static volatile float  vel_command      = 0.0f;
static volatile uint8_t hard_stop_active = 0;

/* ── Position loop divider ───────────────────────────────────────────────── */
static uint8_t pos_div = 0;   /* toggles 0/1; position PID runs when == 0 */

/* ── Trajectory activity (written by vel loop tick, read by pos loop) ─────── */
static volatile uint8_t traj_active   = 0;
static uint8_t prev_traj_active       = 0;

/* ── Settled / ss detection ─────────────────────────────────────────────── */
volatile float    ctrl_pos_err          = 0.0f;
volatile float    ctrl_vel_sp           = 0.0f;
volatile float    ctrl_vel_err          = 0.0f;
volatile float    ctrl_pwm_out          = 0.0f;
volatile float    ctrl_ff_pwm           = 0.0f;
volatile float    ctrl_ideal_pos        = 0.0f;
volatile float    ctrl_ideal_vel        = 0.0f;
volatile uint8_t  ctrl_settled          = 1;
volatile uint8_t  ctrl_model_idx        = 0;

/* Due's deadband thresholds — also exposed to dev_dash.Ctrl */
volatile float ctrl_pos_deadband_deg = 2.0f;   /* |pos_err| < this (°) */
volatile float ctrl_vel_deadband     = 1.0f;   /* |vel| < this (rad/s) for settled */

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void _reset_vel_pid(void) {
    vel_integral   = 0.0f;
    vel_prev_error = 0.0f;
}
static void _reset_pos_pid(void) {
    pos_integral   = 0.0f;
}

/* Due's Velocity_PID_Controller — copied verbatim */
static float _vel_pid(float error, float kp, float ki, float kd,
                      float max_v, float max_pwm_in)
{
    float p_out = kp * error;

    vel_integral += error * CTRL_DT;
    float max_int = (ki > 0.0f) ? (max_pwm_in / ki) : 0.0f;
    if (vel_integral >  max_int) vel_integral =  max_int;
    if (vel_integral < -max_int) vel_integral = -max_int;
    float i_out = ki * vel_integral;

    float derivative = (error - vel_prev_error) / CTRL_DT;
    float d_out      = kd * derivative;
    vel_prev_error   = error;

    float output = p_out + i_out + d_out;
    if (output >  max_pwm_in) output =  max_pwm_in;
    if (output < -max_pwm_in) output = -max_pwm_in;
    return output;
}

/* Due's Position_PID_Controller — copied verbatim */
static float _pos_pid(float pos_error, float vel_feedback,
                      float kp, float ki, float kd, float max_v)
{
    float p_out = kp * pos_error;

    pos_integral += pos_error * CTRL_POS_DT;
    float max_int = (ki > 0.0f) ? (max_v / ki) : 0.0f;
    if (pos_integral >  max_int) pos_integral =  max_int;
    if (pos_integral < -max_int) pos_integral = -max_int;
    float i_out = ki * pos_integral;

    float d_out = -kd * vel_feedback;   /* d/dt(err) = -velocity */

    return p_out + i_out + d_out;
}

/* ── Start_Trajectory — mirrors Due's helper ─────────────────────────────── */
static void _start_traj(float target_rad, float pos_now)
{
    ControlModel_t *m = &control_model[0];
    float disp        = target_rad - pos_now;
    traj_start_pos    = pos_now;
    traj_target       = target_rad;

    if (m->traj_type == 1) {  /* Trapezoid */
        Trapezoid_Init(&g_tr, m->v_max, m->a_max, CTRL_DT);
        Trapezoid_SetTarget(&g_tr, disp);
    } else if (m->traj_type == 2) {  /* Direct */
        g_sc.is_active = 0;
        g_tr.is_active = 0;
    } else {  /* S-Curve (default) */
        SCurve_Init(&g_sc, m->v_max, m->a_max, m->j_max, CTRL_DT);
        SCurve_SetTarget(&g_sc, disp);
    }
    _reset_vel_pid();
    _reset_pos_pid();
    hard_stop_active = 0;
    ctrl_settled     = 0;
}

/* ─────────────────────────────────────────────────────────────────────────── */

void control_init(void)
{
    ControlModel_t *m = &control_model[0];
    SCurve_Init(&g_sc, m->v_max, m->a_max, m->j_max, CTRL_DT);
    Trapezoid_Init(&g_tr, m->v_max, m->a_max, CTRL_DT);

    RefFF_Init(&g_rff,
        0.027f, 0.0012794f, 2.8f, 0.5f, 50.0f, 0.00747f, 0.0083f, 0.02f, CTRL_DT);
    DistFF_Init(&g_dff,
        0.0012794f, 2.8f, 50.0f, 0.00747f, 0.02f, CTRL_DT);

    _reset_vel_pid();
    _reset_pos_pid();
    vel_command      = 0.0f;
    hard_stop_active = 0;
    traj_active      = 0;
    prev_traj_active = 0;
    ctrl_settled     = 1;
}

void control_reset(void)
{
    g_sc.is_active   = 0;
    g_tr.is_active   = 0;
    traj_active      = 0;
    prev_traj_active = 0;
    _reset_vel_pid();
    _reset_pos_pid();
    RefFF_Reset(&g_rff);
    DistFF_Reset(&g_dff);
    vel_command      = 0.0f;
    hard_stop_active = 0;
    ctrl_settled     = 1;
    pos_div          = 0;
}

void control_set_target(float target_rad)
{
    _start_traj(target_rad, robot_pos_rad);
}

/*
 * control_update — called at 1 kHz from TIM7 ISR.
 * pos_rad  : my_encoder.position_rad   (raw windowed, like Due)
 * vel_rad_s: my_encoder.velocity_rad_s (raw windowed, NOT IIR-filtered)
 * pwm_out  : output, -1.0 .. +1.0
 */
void control_update(float pos_rad, float vel_rad_s, float *pwm_out)
{
    ControlModel_t *m = &control_model[0];

    /* ── 1. Advance trajectory at 1 kHz ─────────────────────────────────── */
    float ideal_pos, ideal_vel;
    if (m->traj_type == 1) {
        Trapezoid_Update(&g_tr);
        traj_active = g_tr.is_active;
        ideal_pos   = traj_start_pos + g_tr.p_current;
        ideal_vel   = g_tr.v_current;
    } else if (m->traj_type == 2) {
        traj_active = 0;
        ideal_pos   = traj_target;
        ideal_vel   = 0.0f;
    } else {
        SCurve_Update(&g_sc);
        traj_active = g_sc.is_active;
        ideal_pos   = traj_start_pos + g_sc.p_current;
        ideal_vel   = g_sc.v_current;
    }

    ctrl_ideal_pos = ideal_pos;
    ctrl_ideal_vel = ideal_vel;

    /* ── 2. Position PID at 500 Hz (every 2nd call) ──────────────────────── */
    if (pos_div == 0) {
        /* Trajectory just finished → reset integrators (Due does this too) */
        if (prev_traj_active && !traj_active) {
            _reset_vel_pid();
            _reset_pos_pid();
        }
        prev_traj_active = traj_active;

        float pos_error = ideal_pos - pos_rad;
        ctrl_pos_err    = pos_error;

        float deadband_rad = ctrl_pos_deadband_deg * (3.14159265f / 180.0f);

        if (!traj_active && fabsf(pos_error) <= deadband_rad) {
            /* Settled — Due's hard_stop: freeze motor */
            vel_command      = 0.0f;
            hard_stop_active = 1;
            ctrl_settled     = 1;
        } else {
            hard_stop_active = 0;
            ctrl_settled     = 0;
            float vel_correct = _pos_pid(pos_error, vel_rad_s,
                                         m->kp_pos, m->ki_pos, m->kd_pos,
                                         m->v_max);
            float vel_cmd = ideal_vel + vel_correct;
            if (vel_cmd >  m->v_max) vel_cmd =  m->v_max;
            if (vel_cmd < -m->v_max) vel_cmd = -m->v_max;
            vel_command  = vel_cmd;
            ctrl_vel_sp  = vel_cmd;
        }
    }
    pos_div ^= 1;  /* toggle 0→1→0 */

    /* ── 3. Velocity PID at 1 kHz ────────────────────────────────────────── */
    if (hard_stop_active) {
        _reset_vel_pid();
        *pwm_out      = 0.0f;
        ctrl_pwm_out  = 0.0f;
        return;
    }

    float vel_error = vel_command - vel_rad_s;
    ctrl_vel_err    = vel_error;

    float pwm_pct = _vel_pid(vel_error,
                             m->kp_vel, m->ki_vel, m->kd_vel,
                             m->v_max, 100.0f);

    /* ── 4. Reference Feedforward ────────────────────────────────────────── */
    float ff = 0.0f;
    if (sys_ff_enabled && sys_V_supply > 0.1f) {
        float v_ff = RefFF_Update(&g_rff, vel_command);
        ff = v_ff / sys_V_supply * 100.0f;
        pwm_pct += ff;
    }
    ctrl_ff_pwm = ff;

    if (pwm_pct >  100.0f) pwm_pct =  100.0f;
    if (pwm_pct < -100.0f) pwm_pct = -100.0f;

    *pwm_out     = pwm_pct / 100.0f;   /* convert % → -1..+1 for main.c */
    ctrl_pwm_out = pwm_pct;
}

uint8_t control_is_settled(float pos_rad)
{
    (void)pos_rad;
    return ctrl_settled;
}
