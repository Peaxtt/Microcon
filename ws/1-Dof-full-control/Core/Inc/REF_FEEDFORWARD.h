#ifndef REF_FEEDFORWARD_H
#define REF_FEEDFORWARD_H

#include <stdint.h>

/*
 * Reference Feedforward Filter  —  Model-based voltage feedforward
 *
 * Continuous transfer function (Tustin-discretised):
 *
 *                  n2 s^2  +  n1 s  +  n0
 *   G_ff(s)  =  ──────────────────────────────
 *                  d2 s^2  +  d1 s  +  d0
 *
 * Continuous coefficients (computed once in RefFF_Init):
 *   n2 = J_eq * L
 *   n1 = J_eq * R  +  b_eq * L
 *   n0 = b_eq * R  +  N^2 * Kt * Ke
 *
 *   d2 = N * Kt * tau^2
 *   d1 = 2 * N * Kt * tau
 *   d0 = N * Kt
 *
 * Tustin substitution  (c = 2 / Ts):
 *   b0 =  n2*c^2 + n1*c + n0          a0 =  d2*c^2 + d1*c + d0
 *   b1 = -2*n2*c^2       + 2*n0       a1 = -2*d2*c^2       + 2*d0
 *   b2 =  n2*c^2 - n1*c + n0          a2 =  d2*c^2 - d1*c + d0
 *
 * Difference equation (called every Ts):
 *   y[k] = ( b0*u[k] + b1*u[k-1] + b2*u[k-2]
 *            - a1*y[k-1] - a2*y[k-2] ) / a0
 *
 *   u[k]  —  omega_ref  (rad/s)  velocity from trajectory generator
 *   y[k]  —  V_ff       (V)      feedforward voltage output
 *
 * Converting V_ff to PWM duty (%):
 *   pwm_ff (%) = V_ff / V_supply * 100
 *   Add pwm_ff to the velocity PID output before Set_Motor_PWM().
 *
 * Motor / system parameters:
 *   J_eq   kg·m²      equivalent moment of inertia  (motor + load referred to output)
 *   L      H          armature inductance
 *   R      Ω          armature resistance
 *   b_eq   N·m·s/rad  equivalent viscous friction
 *   N      —          gear ratio  (output_speed = motor_speed / N)
 *   Kt     N·m/A      motor torque constant
 *   Ke     V·s/rad    back-EMF constant
 *   tau    s          filter time constant  (denominator low-pass shape)
 *   Ts     s          sampling period  (= LOOP_DT for TIM6, 0.001 s)
 */

typedef struct {
    /* ── Motor / system parameters ───────────────────────────── */
    float J_eq;   /* Equivalent inertia          (kg·m²)      */
    float L;      /* Armature inductance         (H)           */
    float R;      /* Armature resistance         (Ω)           */
    float b_eq;   /* Equivalent viscous friction (N·m·s/rad)  */
    float N;      /* Gear ratio                  (—)           */
    float Kt;     /* Torque constant             (N·m/A)       */
    float Ke;     /* Back-EMF constant           (V·s/rad)     */
    float tau;    /* Filter time constant        (s)           */
    float Ts;     /* Sampling period             (s)           */

    /* ── Discrete coefficients (computed by RefFF_Init) ──────── */
    float b0, b1, b2;   /* Numerator   coefficients */
    float a0, a1, a2;   /* Denominator coefficients */

    /* ── Delay-line state ─────────────────────────────────────── */
    float u1;   /* u[k-1]  previous input   (rad/s) */
    float u2;   /* u[k-2]  two-step-old input        */
    float y1;   /* y[k-1]  previous output  (V)      */
    float y2;   /* y[k-2]  two-step-old output       */
} RefFF_t;

/*
 * RefFF_Init — compute discrete coefficients from motor parameters.
 * Call once at startup (or after parameter change).
 */
void RefFF_Init(RefFF_t *ff,
                float J_eq, float L, float R, float b_eq,
                float N,    float Kt, float Ke,
                float tau,  float Ts);

/*
 * RefFF_Reset — zero the delay-line state.
 * Call whenever the trajectory restarts or the motor stops.
 */
void RefFF_Reset(RefFF_t *ff);

/*
 * RefFF_Update — run one step of the difference equation.
 *   omega_ref  velocity setpoint from trajectory generator  (rad/s)
 *   returns    feedforward voltage  V_ff                    (V)
 *
 * Call at the inner-loop rate (TIM6, 1 kHz) with Ts = LOOP_DT.
 */
float RefFF_Update(RefFF_t *ff, float omega_ref);

#endif /* REF_FEEDFORWARD_H */
