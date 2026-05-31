#ifndef __CONTROL_H
#define __CONTROL_H

/*
 * control.h — Control Layer Interface
 * =====================================
 * Declares the API that the control module must implement.
 * This header is owned by the firmware team.
 * The control team provides control.c (PID + trajectory).
 *
 * ── Model selection ──────────────────────────────────────────────────────
 * The control module selects the appropriate model automatically:
 *   |target - current| < control_model_switch_deg  →  control_model[0]  (fine)
 *   |target - current| ≥ control_model_switch_deg  →  control_model[1]  (coarse)
 *
 * Both models are configured in robot.h as control_model[CONTROL_MODEL_COUNT].
 * Set via Live Expressions or Modbus registers 0x0C–0x0F, 0x38–0x3F.
 *
 * ── Default behaviour when control.c is not linked ───────────────────────
 * All functions are __weak. Defaults:
 *   control_update()     → *pwm_out = 0.0f  (motor off)
 *   control_is_settled() → returns 1         (FSM can still transition)
 *
 * ── Integration ───────────────────────────────────────────────────────────
 *   main.c USER CODE 2 : control_init()
 *   ISR (1kHz)         : control_update(pos, vel, &pwm)
 *   state_machine.c    : control_set_target(), control_reset(), control_is_settled()
 */

#include "robot.h"

/* ── Lifecycle ────────────────────────────────────────────────────────────── */

/* Called once at startup, after encoder is initialised. */
void    control_init(void);

/* Called when entering a motion state (AUTO, SEQUENCE, TEST, MANUAL_SWITCH).
 * Clears integrators, resets trajectory. */
void    control_reset(void);

/* ── Per-move ─────────────────────────────────────────────────────────────── */

/* Arm a new move to target_rad. Called from state_machine_tick (100 Hz).
 * Selects model based on |target - current| vs control_model_switch_deg. */
void    control_set_target(float target_rad);

/* ── Per-tick (1 kHz ISR) ─────────────────────────────────────────────────── */

/* One control cycle.
 *   pos_rad    — current encoder position (rad)
 *   vel_rad_s  — filtered encoder velocity (rad/s)
 *   pwm_out    — write commanded PWM fraction here  (-1.0 .. +1.0)
 */
void    control_update(float pos_rad, float vel_rad_s, float *pwm_out);

/* ── Status ───────────────────────────────────────────────────────────────── */

/* Returns 1 when the move is complete (trajectory done + position settled).
 * Called from state_machine_tick to advance sequence steps. */
uint8_t control_is_settled(float pos_rad);

#endif /* __CONTROL_H */
