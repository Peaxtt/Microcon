#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <stdint.h>
#include <math.h>

/*
 * 3-Segment Trapezoidal velocity profile generator.
 *
 * Given a displacement, it pre-computes a symmetric profile that:
 *   - Starts at v = 0
 *   - Accelerates at constant a_max through segment 1
 *   - Cruises at v_peak through segment 2 (may have zero duration)
 *   - Decelerates at constant -a_max through segment 3
 *   - Ends at v = 0
 *
 * Segment acceleration schedule:
 *   1: a = +a_max   (v: 0      -> v_peak)
 *   2: a =  0       (v: v_peak -> v_peak) [may be zero-length]
 *   3: a = -a_max   (v: v_peak -> 0)
 */
typedef struct {
    /* Configuration */
    float v_max;    /* Maximum cruise velocity  (rad/s)   */
    float a_max;    /* Maximum acceleration     (rad/s^2) */
    float dt;       /* Control loop time step   (s)       */

    /* Pre-computed profile (set by Trapezoid_SetTarget) */
    float v_peak;   /* Actual peak velocity used         (rad/s) */
    float T1;       /* End of acceleration segment       (s)     */
    float T2;       /* End of cruise segment             (s)     */
    float T3;       /* End of deceleration segment — total time  */
    float dir;      /* Direction: +1.0 or -1.0                   */

    /* Runtime state */
    float t_now;        /* Elapsed time since move start (s)       */

    /* Runtime outputs (updated every Trapezoid_Update call) */
    float v_current;    /* Current profile velocity      (rad/s)   */
    float a_current;    /* Current profile acceleration  (rad/s^2) */
    float p_current;    /* Integrated ideal position     (rad)     */

    uint8_t is_active;  /* 1 while profile is running, 0 when done */
} Trapezoid_t;

void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt);

/* Constraint-based: computes segment times from v_max, a_max */
void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement);

/* Time-based: you specify each segment duration; v_peak and a_max are
 * back-computed so the profile covers exactly the given displacement.
 *   t_acc    — duration of acceleration segment (seg 1)  (s)
 *   t_cruise — duration of cruise segment       (seg 2)  (s)
 * Deceleration is always symmetric with t_acc.            */
void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement,
                                 float t_acc, float t_cruise);

void Trapezoid_Update(Trapezoid_t *tr);

#endif /* TRAPEZOID_H */
