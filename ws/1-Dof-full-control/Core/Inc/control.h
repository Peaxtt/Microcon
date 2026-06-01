#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include <stdint.h>

/* ── Control model (PID gains + trajectory params) ───────────────────────── */
typedef struct {
  float kp_vel, ki_vel, kd_vel;   /* velocity PID */
  float kp_pos, ki_pos, kd_pos;   /* position PID */
  float v_max, a_max, j_max;      /* trajectory (rad/s, rad/s², rad/s³) */
  uint8_t traj_type;              /* 0=trapezoid  1=s-curve  2=direct */
} ControlModel_t;

#define CONTROL_MODEL_COUNT  2

/* ── Globals provided by main.c (consumed by control.c) ─────────────────── */
extern ControlModel_t     control_model[CONTROL_MODEL_COUNT];
extern volatile float     control_model_switch_deg;
extern volatile float     robot_pos_rad;     /* updated every 1kHz tick */
extern volatile uint8_t   sys_ff_enabled;    /* 1 = reference feedforward on */
extern volatile float     sys_V_supply;      /* motor bus voltage (V) */

/* ── Observables written by control.c (read via Live Expressions) ────────── */
extern volatile float    ctrl_pos_err;
extern volatile float    ctrl_vel_sp;
extern volatile float    ctrl_vel_err;
extern volatile float    ctrl_pwm_out;
extern volatile float    ctrl_ff_pwm;
extern volatile float    ctrl_ideal_pos;
extern volatile float    ctrl_ideal_vel;
extern volatile uint8_t  ctrl_settled;
extern volatile uint8_t  ctrl_model_idx;
extern volatile float    ctrl_pos_deadband_deg;
extern volatile float    ctrl_vel_deadband;

/* ── Interface ───────────────────────────────────────────────────────────── */
void    control_init(void);
void    control_reset(void);
void    control_set_target(float target_rad);
void    control_update(float pos_rad, float vel_rad_s, float *pwm_out);
uint8_t control_is_settled(float pos_rad);

#endif /* __CONTROL_H */
