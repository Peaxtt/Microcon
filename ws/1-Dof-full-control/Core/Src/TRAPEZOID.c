#include "trapezoid.h"

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt) {
    tr->v_max = v_max;
    tr->a_max = a_max;
    tr->dt    = dt;

    tr->v_peak    = 0.0f;
    tr->T1        = 0.0f;
    tr->T2        = 0.0f;
    tr->T3        = 0.0f;
    tr->dir       = 1.0f;
    tr->t_now     = 0.0f;
    tr->v_current = 0.0f;
    tr->a_current = 0.0f;
    tr->p_current = 0.0f;
    tr->is_active = 0;
}

void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement) {
    if (fabsf(displacement) < 0.001f) return;

    tr->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    float d       = fabsf(displacement);

    tr->t_now     = 0.0f;
    tr->v_current = 0.0f;
    tr->a_current = 0.0f;
    tr->p_current = 0.0f;
    tr->is_active = 1;

    /* v_peak = min(v_max, sqrt(a_max * d))
     * sqrt(a*d) is the peak velocity achievable if there is no cruise phase
     * (triangle profile — both halves are pure acceleration/deceleration). */
    float v_peak = sqrtf(tr->a_max * d);
    if (v_peak > tr->v_max) v_peak = tr->v_max;
    tr->v_peak = v_peak;

    /* Segment durations */
    float t_acc    = v_peak / tr->a_max;
    float d_acc    = 0.5f * tr->a_max * t_acc * t_acc;   /* = v_peak^2 / (2*a_max) */
    float d_cruise = d - 2.0f * d_acc;
    float t_cruise = (d_cruise > 0.0f) ? d_cruise / v_peak : 0.0f;

    tr->T1 = t_acc;
    tr->T2 = tr->T1 + t_cruise;
    tr->T3 = tr->T2 + t_acc;
}

void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement,
                                 float t_acc, float t_cruise) {
    if (fabsf(displacement) < 0.001f) return;
    if (t_acc <= 0.0f) return;

    tr->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    float d       = fabsf(displacement);

    tr->t_now     = 0.0f;
    tr->v_current = 0.0f;
    tr->a_current = 0.0f;
    tr->p_current = 0.0f;
    tr->is_active = 1;

    /* Back-compute v_peak from segment times.
     *
     * Area identity:  d = v_peak × (t_acc + t_cruise)
     *   (deceleration is symmetric so total accel distance = v_peak * t_acc)
     *
     * Then:  a_used = v_peak / t_acc
     */
    float v_peak = d / (t_acc + t_cruise);
    tr->v_peak   = v_peak;
    tr->a_max    = v_peak / t_acc;   /* override stored a_max for this move */

    tr->T1 = t_acc;
    tr->T2 = t_acc + t_cruise;
    tr->T3 = t_acc + t_cruise + t_acc;
}

void Trapezoid_Update(Trapezoid_t *tr) {
    if (!tr->is_active) {
        tr->v_current = 0.0f;
        tr->a_current = 0.0f;
        return;
    }

    float t = tr->t_now;
    float v, a;

    if (t < tr->T1) {
        /* Segment 1 — constant acceleration */
        a = tr->a_max;
        v = tr->a_max * t;
    }
    else if (t < tr->T2) {
        /* Segment 2 — cruise at v_peak */
        a = 0.0f;
        v = tr->v_peak;
    }
    else if (t < tr->T3) {
        /* Segment 3 — constant deceleration */
        float tau = t - tr->T2;
        a = -tr->a_max;
        v = tr->v_peak - tr->a_max * tau;
    }
    else {
        /* Profile complete */
        a             = 0.0f;
        v             = 0.0f;
        tr->is_active = 0;
    }

    tr->v_current  = v * tr->dir;
    tr->a_current  = a * tr->dir;
    tr->p_current += tr->v_current * tr->dt;
    tr->t_now     += tr->dt;
}
