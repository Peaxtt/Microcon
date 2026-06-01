#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <stdint.h>
#include <math.h>

/*
 * 3-Segment Asymmetric Trapezoidal velocity profile generator.
 *
 * The user specifies all three segment durations directly (t_acc, t_cruise, t_dec).
 * v_peak is back-computed from the displacement and the given times.
 * v_max acts as a safety ceiling only — it does NOT determine t_cruise.
 *
 * Segment schedule:
 *   1: a = +a_acc   (v: 0       → v_peak)   duration t_acc
 *   2: a =  0       (v: v_peak  → v_peak)   duration t_cruise  (may be 0)
 *   3: a = -a_dec   (v: v_peak  → 0)        duration t_dec
 *
 * Displacement identity (asymmetric):
 *   d = 0.5*v_peak*t_acc  +  v_peak*t_cruise  +  0.5*v_peak*t_dec
 *     = v_peak * (t_acc/2  +  t_cruise  +  t_dec/2)
 *
 *   v_peak = d / (t_acc/2 + t_cruise + t_dec/2)
 *
 * Clamping:
 *   If the computed v_peak > v_max, v_peak is clamped to v_max.
 *   t_acc, t_cruise, t_dec remain unchanged.
 *   a_acc and a_dec are recomputed from the clamped v_peak.
 */
typedef struct {
    /* Configuration */
    float v_max;    /* Safety ceiling velocity  (rad/s)   */
    float a_max;    /* Max acceleration limit   (rad/s^2) — used by SetTarget (constraint) */
    float dt;       /* Control loop time step   (s)       */

    /* Pre-computed profile (set by SetTarget / SetTarget_ByTime) */
    float v_peak;   /* Actual peak velocity              (rad/s)   */
    float a_acc;    /* Acceleration rate for segment 1   (rad/s^2) */
    float a_dec;    /* Deceleration rate for segment 3   (rad/s^2) */
    float T1;       /* End of acceleration segment       (s)       */
    float T2;       /* End of cruise segment             (s)       */
    float T3;       /* End of deceleration segment       (s)       */
    float dir;      /* Direction: +1.0 or -1.0                     */

    /* Runtime state */
    float t_now;        /* Elapsed time since move start (s)       */

    /* Runtime outputs (updated every Trapezoid_Update call) */
    float v_current;    /* Current profile velocity      (rad/s)   */
    float a_current;    /* Current profile acceleration  (rad/s^2) */
    float p_current;    /* Integrated ideal position     (rad)     */

    uint8_t is_active;  /* 1 while profile is running, 0 when done */
} Trapezoid_t;

void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt);

/* Constraint-based: computes symmetric segment times from v_max, a_max */
void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement);

/*
 * Time-based: user specifies all three segment durations.
 *
 *   t_acc    — acceleration segment duration  (s)  — segment 1
 *   t_cruise — cruise segment duration        (s)  — segment 2  (0 = no cruise)
 *   t_dec    — deceleration segment duration  (s)  — segment 3
 *
 * v_peak is back-computed:  v_peak = |d| / (t_acc/2 + t_cruise + t_dec/2)
 * v_peak is clamped to v_max only if it exceeds v_max.
 * t_cruise is NEVER recomputed from v_max.
 */
void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement,
                                 float t_acc, float t_cruise, float t_dec);

void Trapezoid_Update(Trapezoid_t *tr);

#endif /* TRAPEZOID_H */
