#include "scurve.h"

/* -----------------------------------------------------------------------
 * Internal helper: pre-compute all 7 segment boundary times for a move
 * of magnitude |d| (always positive; direction is stored in sc->dir).
 * ----------------------------------------------------------------------- */
static void Compute_Profile(SCurve_t *sc, float d) {
    float j = sc->j_max;
    float v = sc->v_max;

    /* --- Constant-jerk profile: t2 = 0 always ---
     *
     * Jerk is ±j_max throughout; acceleration forms a triangle (not trapezoid).
     *   t1    = sqrt(v_peak / j_max)  — duration of each jerk segment
     *   a_peak = j_max × t1 = sqrt(j_max × v_peak)
     *   d_acc  = v_peak × t1          — distance over both jerk segments
     */
    float t1_used = sqrtf(v / j);
    float v_peak  = v;
    float a_peak  = j * t1_used;

    float t_acc = 2.0f * t1_used;
    float d_acc = v_peak * t_acc / 2.0f;   /* = v_peak * t1_used */

    /* --- Reduce v_peak if the displacement is too short for a cruise ---
     * With t2 = 0:  d = 2 × v_peak^(3/2) / sqrt(j_max)
     *              v_peak = (d × sqrt(j_max) / 2) ^ (2/3)              */
    if (d < 2.0f * d_acc) {
        v_peak  = powf(d * sqrtf(j) / 2.0f, 2.0f / 3.0f);
        t1_used = sqrtf(v_peak / j);
        a_peak  = j * t1_used;
        t_acc   = 2.0f * t1_used;
        d_acc   = v_peak * t1_used;
    }

    sc->v_peak = v_peak;
    sc->a_peak = a_peak;

    /* Cruise duration */
    float t_cruise = (d - 2.0f * d_acc) / v_peak;
    if (t_cruise < 0.0f) t_cruise = 0.0f;

    /* Cumulative segment boundary times (T[2]=T[1], T[6]=T[5] — segs 2&6 zero-length) */
    sc->T[0] = 0.0f;
    sc->T[1] = t1_used;                  /* end seg 1: +j_max */
    sc->T[2] = t1_used;                  /* seg 2 skipped (t2=0) */
    sc->T[3] = 2.0f * t1_used;          /* end seg 3: -j_max */
    sc->T[4] = sc->T[3] + t_cruise;     /* end seg 4: cruise  */
    sc->T[5] = sc->T[4] + t1_used;      /* end seg 5: -j_max */
    sc->T[6] = sc->T[5];                 /* seg 6 skipped (t2=0) */
    sc->T[7] = sc->T[6] + t1_used;      /* end seg 7: +j_max */
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max, float dt) {
    sc->v_max = v_max;
    sc->a_max = a_max;
    sc->j_max = j_max;
    sc->dt    = dt;

    for (int i = 0; i < 8; i++) sc->T[i] = 0.0f;
    sc->v_peak   = 0.0f;
    sc->a_peak   = 0.0f;
    sc->dir      = 1.0f;
    sc->t_now    = 0.0f;     /* declared below — add to struct */
    sc->v_current = 0.0f;
    sc->a_current = 0.0f;
    sc->j_current = 0.0f;
    sc->p_current = 0.0f;
    sc->is_active = 0;
}

void SCurve_SetTarget(SCurve_t *sc, float displacement) {
    if (fabsf(displacement) < 0.001f) return;

    sc->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    sc->t_now     = 0.0f;
    sc->v_current = 0.0f;
    sc->a_current = 0.0f;
    sc->j_current = 0.0f;
    sc->p_current = 0.0f;
    sc->is_active = 1;

    Compute_Profile(sc, fabsf(displacement));
}

void SCurve_SetTarget_ByTime(SCurve_t *sc, float displacement,
                              float t1, float t2, float t_cruise) {
    if (fabsf(displacement) < 0.001f) return;
    if (t1 <= 0.0f) return;

    sc->dir       = (displacement > 0.0f) ? 1.0f : -1.0f;
    float d       = fabsf(displacement);

    sc->t_now     = 0.0f;
    sc->v_current = 0.0f;
    sc->a_current = 0.0f;
    sc->j_current = 0.0f;
    sc->p_current = 0.0f;
    sc->is_active = 1;

    /* Full 7-segment time-based profile with optional t2 constant-accel phase.
     *
     * Area identity:  d = v_peak × (2×t1 + t2 + t_cruise)
     *   v_peak = d / (2×t1 + t2 + t_cruise)
     *   a_peak = v_peak / (t1 + t2)
     *
     * When t2 = 0 the profile degenerates to the pure constant-jerk triangle
     * (same as before). Set t2 > 0 to get a flat-top acceleration plateau.
     */
    float t_acc  = 2.0f * t1 + t2;
    float v_peak = d / (t_acc + t_cruise);
    float a_peak = (t1 + t2 > 1e-6f) ? (v_peak / (t1 + t2)) : 0.0f;

    sc->v_peak = v_peak;
    sc->a_peak = a_peak;

    /* Segment boundary times (T[2]=T[1]+t2, T[6]=T[5]+t2; zero when t2=0) */
    sc->T[0] = 0.0f;
    sc->T[1] = t1;
    sc->T[2] = t1 + t2;
    sc->T[3] = 2.0f * t1 + t2;
    sc->T[4] = sc->T[3] + t_cruise;
    sc->T[5] = sc->T[4] + t1;
    sc->T[6] = sc->T[5] + t2;
    sc->T[7] = sc->T[6] + t1;
}

void SCurve_Update(SCurve_t *sc) {
    if (!sc->is_active) {
        sc->v_current = 0.0f;
        sc->a_current = 0.0f;
        sc->j_current = 0.0f;
        return;
    }

    float t  = sc->t_now;
    /* Derive actual jerk from stored a_peak and T[1] — works for both
     * SetTarget (constraint-based) and SetTarget_ByTime.             */
    float j  = (sc->T[1] > 1e-6f) ? sc->a_peak / sc->T[1] : sc->j_max;
    float ap = sc->a_peak;
    float vp = sc->v_peak;
    float t1 = sc->T[1];          /* jerk-segment duration */
    float t2 = sc->T[2] - sc->T[1];  /* const-accel duration  */

    /* Pre-compute velocity checkpoints (positive direction) */
    float v1 = j * t1 * t1 / 2.0f;               /* end of seg 1            */
    float v2 = v1 + ap * t2;                      /* end of seg 2            */
    float v5 = vp - v1;                           /* end of seg 5            */
    float v6 = v1;                                /* end of seg 6 (= v1 sym) */

    float v, a, jerk, tau;

    if (t < sc->T[1]) {
        /* Segment 1 — jerk = +j_max */
        tau  = t;
        jerk = +j;
        a    =  j * tau;
        v    =  j * tau*tau / 2.0f;
    }
    else if (t < sc->T[2]) {
        /* Segment 2 — constant acceleration a_peak */
        tau  = t - sc->T[1];
        jerk = 0.0f;
        a    = ap;
        v    = v1 + ap * tau;
    }
    else if (t < sc->T[3]) {
        /* Segment 3 — jerk = -j_max, a decreases to 0 */
        tau  = t - sc->T[2];
        jerk = -j;
        a    = ap - j * tau;
        v    = v2 + ap * tau - j * tau*tau / 2.0f;
    }
    else if (t < sc->T[4]) {
        /* Segment 4 — cruise at v_peak */
        jerk = 0.0f;
        a    = 0.0f;
        v    = vp;
    }
    else if (t < sc->T[5]) {
        /* Segment 5 — jerk = -j_max, a goes 0 -> -a_peak */
        tau  = t - sc->T[4];
        jerk = -j;
        a    = -j * tau;
        v    = vp - j * tau*tau / 2.0f;
    }
    else if (t < sc->T[6]) {
        /* Segment 6 — constant deceleration -a_peak */
        tau  = t - sc->T[5];
        jerk = 0.0f;
        a    = -ap;
        v    = v5 - ap * tau;
    }
    else if (t < sc->T[7]) {
        /* Segment 7 — jerk = +j_max, a returns -a_peak -> 0 */
        tau  = t - sc->T[6];
        jerk = +j;
        a    = -ap + j * tau;
        v    = v6 - ap * tau + j * tau*tau / 2.0f;
    }
    else {
        /* Profile complete */
        jerk          = 0.0f;
        a             = 0.0f;
        v             = 0.0f;
        sc->is_active = 0;
    }

    sc->j_current  = jerk * sc->dir;
    sc->a_current  = a    * sc->dir;
    sc->v_current  = v    * sc->dir;
    sc->p_current += sc->v_current * sc->dt;
    sc->t_now     += sc->dt;
}
