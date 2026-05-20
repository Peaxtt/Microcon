#ifndef SCURVE_H
#define SCURVE_H

#include <stdint.h>
#include <math.h>

/*
 * Constant-jerk S-Curve velocity profile generator (5 active segments).
 *
 * Jerk is always ±j_max — never zero during acceleration or deceleration.
 * Acceleration forms a triangle (not a trapezoid): rises linearly to a_peak
 * then falls immediately, giving a smooth parabolic velocity transition.
 *
 * Profile shape (positive direction):
 *   Seg 1: j = +j_max   a: 0  → a_peak   v: 0       → v_peak/2
 *   Seg 3: j = -j_max   a: a_peak → 0    v: v_peak/2 → v_peak
 *   Seg 4: j =  0       cruise at v_peak  [may be zero-length]
 *   Seg 5: j = -j_max   a: 0  →-a_peak   v: v_peak → v_peak/2
 *   Seg 7: j = +j_max   a:-a_peak → 0    v: v_peak/2 → 0
 *
 * Segments 2 & 6 (constant acceleration) are always zero-length.
 * Relationships:  t1 = sqrt(v_peak / j_max)
 *                 a_peak = j_max × t1 = sqrt(j_max × v_peak)
 */
typedef struct {
    /* Configuration */
    float v_max;    /* Maximum cruise velocity   (rad/s)   */
    float a_max;    /* Maximum acceleration      (rad/s^2) */
    float j_max;    /* Maximum jerk              (rad/s^3) */
    float dt;       /* Control loop time step    (s)       */

    /* Pre-computed profile (set by SCurve_SetTarget) */
    float T[8];     /* Cumulative segment end times; T[0]=0, T[7]=total_time */
    float v_peak;   /* Actual peak velocity achieved  (rad/s)   */
    float a_peak;   /* Actual peak acceleration used  (rad/s^2) */
    float dir;      /* Direction: +1.0 or -1.0                  */

    /* Runtime state */
    float t_now;        /* Elapsed time since move start (s)       */

    /* Runtime outputs (updated every SCurve_Update call) */
    float v_current;    /* Current profile velocity      (rad/s)   */
    float a_current;    /* Current profile acceleration  (rad/s^2) */
    float j_current;    /* Current profile jerk          (rad/s^3) */
    float p_current;    /* Integrated ideal position     (rad)     */

    uint8_t is_active;  /* 1 while profile is running, 0 when done */
} SCurve_t;

void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max, float dt);

/* Constraint-based: computes segment times from v_max, a_max, j_max */
void SCurve_SetTarget(SCurve_t *sc, float displacement);

/* Time-based: you specify segment durations; v_peak and a_peak are
 * back-computed so the profile covers exactly the given displacement.
 *   t1       — duration of each jerk segment (segs 1,3,5,7)  (s)
 *   t2       — IGNORED (constant-jerk profile forces t2 = 0)
 *   t_cruise — duration of the cruise phase (seg 4)           (s) */
void SCurve_SetTarget_ByTime(SCurve_t *sc, float displacement,
                              float t1, float t2, float t_cruise);

void SCurve_Update(SCurve_t *sc);

#endif /* SCURVE_H */
