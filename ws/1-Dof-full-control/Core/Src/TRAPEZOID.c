#include "trapezoid.h"

void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt) {
    tr->v_max = v_max;
    tr->a_max = a_max;
    tr->dt    = dt;

    tr->v_peak    = 0.0f;
    tr->a_acc     = 0.0f;
    tr->a_dec     = 0.0f;
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

/* -----------------------------------------------------------------------
 * Constraint-based: symmetric profile from v_max, a_max.
 * ----------------------------------------------------------------------- */
void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement) {
    if (fabsf(displacement) < 0.001f) return;

    tr->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    float d       = fabsf(displacement);

    tr->t_now     = 0.0f;
    tr->v_current = 0.0f;
    tr->a_current = 0.0f;
    tr->p_current = 0.0f;
    tr->is_active = 1;

    /* v_peak = min(v_max, sqrt(a_max * d)) */
    float v_peak = sqrtf(tr->a_max * d);
    if (v_peak > tr->v_max) v_peak = tr->v_max;
    tr->v_peak = v_peak;

    float t_acc    = v_peak / tr->a_max;
    float d_acc    = 0.5f * tr->a_max * t_acc * t_acc;
    float d_cruise = d - 2.0f * d_acc;
    float t_cruise = (d_cruise > 0.0f) ? d_cruise / v_peak : 0.0f;

    tr->a_acc = tr->a_max;   /* symmetric: accel = decel = a_max */
    tr->a_dec = tr->a_max;

    tr->T1 = t_acc;
    tr->T2 = t_acc + t_cruise;
    tr->T3 = t_acc + t_cruise + t_acc;   /* symmetric decel duration */
}

/* -----------------------------------------------------------------------
 * Time-based: user specifies t_acc, t_cruise, t_dec directly.
 *
 * v_peak = |d| / (t_acc/2 + t_cruise + t_dec/2)
 *
 * Clamping: if v_peak > v_max  →  v_peak = v_max.
 *           t_acc, t_cruise, t_dec are never changed.
 *           a_acc and a_dec are recomputed from the (possibly clamped) v_peak.
 * ----------------------------------------------------------------------- */
void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement,
                                 float t_acc, float t_cruise, float t_dec) {
    if (fabsf(displacement) < 0.001f) return;
    if (t_acc <= 0.0f || t_dec <= 0.0f) return;

    tr->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    float d       = fabsf(displacement);

    tr->t_now     = 0.0f;
    tr->v_current = 0.0f;
    tr->a_current = 0.0f;
    tr->p_current = 0.0f;
    tr->is_active = 1;

    /* Asymmetric area identity:
     *   d = 0.5*v_peak*t_acc  +  v_peak*t_cruise  +  0.5*v_peak*t_dec
     *     = v_peak * (t_acc/2  +  t_cruise  +  t_dec/2)
     */
    float denom  = 0.5f * t_acc + t_cruise + 0.5f * t_dec;
    float v_peak = (denom > 1e-6f) ? (d / denom) : 0.0f;

    /* Clamp to v_max only when the back-computed velocity exceeds the limit.
     * In the clamped case, extend t_cruise to still cover the full displacement:
     *   d = v_max * (t_acc/2 + t_cruise_new + t_dec/2)
     *   t_cruise_new = d/v_max - t_acc/2 - t_dec/2
     * The user's t_cruise is used unchanged when no clamping occurs. */
    if (v_peak > tr->v_max) {
        v_peak   = tr->v_max;
        t_cruise = d / tr->v_max - 0.5f * t_acc - 0.5f * t_dec;
        if (t_cruise < 0.0f) t_cruise = 0.0f;
    }

    tr->v_peak = v_peak;
    tr->a_acc  = v_peak / t_acc;   /* accel rate for segment 1 */
    tr->a_dec  = v_peak / t_dec;   /* decel rate for segment 3 */

    tr->T1 = t_acc;
    tr->T2 = t_acc + t_cruise;
    tr->T3 = t_acc + t_cruise + t_dec;
}

/* -----------------------------------------------------------------------
 * Update — call every dt.
 * Segment 1 uses a_acc; segment 3 uses a_dec (may differ).
 * ----------------------------------------------------------------------- */
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
        a = tr->a_acc;
        v = tr->a_acc * t;
    }
    else if (t < tr->T2) {
        /* Segment 2 — cruise at v_peak */
        a = 0.0f;
        v = tr->v_peak;
    }
    else if (t < tr->T3) {
        /* Segment 3 — constant deceleration (rate = a_dec, may differ from a_acc) */
        float tau = t - tr->T2;
        a = -tr->a_dec;
        v = tr->v_peak - tr->a_dec * tau;
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
