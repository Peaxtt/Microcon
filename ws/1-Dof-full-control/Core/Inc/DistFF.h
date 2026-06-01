/*
 * DistFF.h
 *
 * Disturbance Feedforward Filter  —  1st-order Tustin-discretised.
 *
 * Converts the Kalman4 disturbance torque estimate (tau_d_hat, N·m)
 * into a compensating voltage (V_ff_d, V) that is added to the
 * velocity PID output before Set_Motor_PWM().
 *
 * ── Continuous transfer function ──────────────────────────────────────
 *
 *                  n1·s + n0
 *   G_ff(s) = ──────────────────
 *                  d1·s + d0
 *
 *   n1 = L                   (armature inductance, H)
 *   n0 = R                   (armature resistance, Ω)
 *   d1 = N · Kt · tau        (gear ratio × torque constant × filter τ)
 *   d0 = N · Kt
 *
 * ── Tustin discretisation  (c = 2/Ts) ────────────────────────────────
 *
 *   b0 =  n1·c + n0          a0 =  d1·c + d0
 *   b1 = -n1·c + n0          a1 = -d1·c + d0
 *
 * ── Difference equation ───────────────────────────────────────────────
 *
 *   y[k] = ( b0·u[k] + b1·u[k-1] - a1·y[k-1] ) / a0
 *
 *   u[k]  — tau_d_hat  (N·m)  estimated disturbance from Kalman4 x[3]
 *   y[k]  — V_ff_d     (V)    feedforward voltage output
 *
 * ── Converting V_ff_d to PWM ──────────────────────────────────────────
 *
 *   pwm_ff_d (%) = V_ff_d / V_supply × 100
 *   Add pwm_ff_d to the velocity PID output before Set_Motor_PWM().
 *
 * ── Parameters ────────────────────────────────────────────────────────
 *
 *   L     H           armature inductance
 *   R     Ω           armature resistance
 *   N     —           gear ratio  (motor → load)
 *   Kt    N·m/A       motor torque constant
 *   tau   s           filter time constant for the denominator
 *   Ts    s           sampling period (= LOOP_DT = 0.001 s)
 */

#ifndef INC_DISTFF_H_
#define INC_DISTFF_H_

typedef struct {
    /* Motor / system parameters */
    float L;     /* armature inductance         (H)          */
    float R;     /* armature resistance         (Ω)          */
    float N;     /* gear ratio                  (—)          */
    float Kt;    /* torque constant             (N·m/A)      */
    float tau;   /* filter time constant        (s)          */
    float Ts;    /* sampling period             (s)          */

    /* Discrete coefficients (computed once in DistFF_Init) */
    float b0, b1;   /* numerator   */
    float a0, a1;   /* denominator */

    /* Delay-line state */
    float u1;   /* u[k-1]  previous disturbance input  (N·m) */
    float y1;   /* y[k-1]  previous voltage output      (V)  */
} DistFF_t;

/*
 * DistFF_Init — compute discrete coefficients from motor parameters.
 * Call once at startup, or again after parameter changes.
 *
 *   tau  — filter time constant in the denominator (same role as tau in RefFF_Init)
 *          NOT the disturbance torque; that is the runtime input to DistFF_Update.
 */
void DistFF_Init(DistFF_t *ff,
                 float L, float R,
                 float N, float Kt,
                 float tau, float Ts);

/*
 * DistFF_Reset — zero the delay-line state.
 * Call when the motor stops, on reset_all, or on mode change.
 */
void DistFF_Reset(DistFF_t *ff);

/*
 * DistFF_Update — one step of the difference equation.
 *
 *   tau_d_hat  estimated disturbance torque from Kalman4 x[3]  (N·m)
 *   returns    feedforward voltage  V_ff_d                      (V)
 *
 * Convert to PWM addend:  pwm_ff_d = V_ff_d / V_supply * 100.0f
 */
float DistFF_Update(DistFF_t *ff, float tau_d_hat);

#endif /* INC_DISTFF_H_ */
