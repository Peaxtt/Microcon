#include "SCURVE.h"

/* -----------------------------------------------------------------------
 * Internal helper: pre-compute all 7 segment boundary times for a move
 * of magnitude |d| (always positive; direction is stored in sc->dir).
 * ----------------------------------------------------------------------- */
static void Compute_Profile(SCurve_t *sc, float d) {
    float j = sc->j_max;
    float a = sc->a_max;
    float v = sc->v_max;

    /* --- Determine peak acceleration and jerk-segment duration (t1) ---
     *
     * t1_full : time to reach a_max at rate j_max
     * v_full  : minimum v_peak that uses the full a_max (segment 2 = 0)
     *           v_full = j_max * t1^2  (velocity gained in 2 jerk segments)
     */
    float t1 = a / j;
    float v_full = j * t1 * t1;   /* = a_max^2 / j_max */

    float t1_used, t2, v_peak, a_peak;

    /* Choose whether the full a_max is reachable for the given v_max */
    if (v <= v_full) {
        /* v_max is reached before a_max — shorten t1, no segment 2 */
        t1_used = sqrtf(v / j);
        t2      = 0.0f;
        v_peak  = v;
        a_peak  = j * t1_used;
    } else {
        t1_used = t1;
        t2      = (v - v_full) / a;
        v_peak  = v;
        a_peak  = a;
    }

    /* Distance consumed by the full acceleration phase (segs 1+2+3).
     * For any symmetric jerk profile: d_acc = v_peak * t_acc / 2
     * where t_acc = 2*t1 + t2.  (Proven analytically — area under the
     * velocity ramp from 0 to v_peak.) */
    float t_acc = 2.0f * t1_used + t2;
    float d_acc = v_peak * t_acc / 2.0f;

    /* --- Reduce v_peak if the displacement is too short for a cruise --- */
    if (d < 2.0f * d_acc) {
        /* Try keeping full a_max: solve  v^2/a + v*t1 = d
         * (quadratic in v_peak):  v = (-t1 + sqrt(t1^2 + 4d/a)) * a/2  */
        float disc       = t1 * t1 + 4.0f * d / a;
        float v_candidate = (-t1 + sqrtf(disc)) * a / 2.0f;

        if (v_candidate <= v_full) {
            /* Even with t2=0 the distance is too short — reduce a_max too.
             * With t2=0:  d = 2 * v^(3/2) / sqrt(j)
             *             v_peak = (d * sqrt(j) / 2) ^ (2/3)            */
            v_peak  = powf(d * sqrtf(j) / 2.0f, 2.0f / 3.0f);
            t1_used = sqrtf(v_peak / j);
            t2      = 0.0f;
            a_peak  = j * t1_used;
        } else {
            v_peak  = v_candidate;
            t1_used = t1;
            t2      = (v_peak - v_full) / a;
            a_peak  = a;
        }

        t_acc = 2.0f * t1_used + t2;
        d_acc = v_peak * t_acc / 2.0f;
    }

    sc->v_peak = v_peak;
    sc->a_peak = a_peak;

    /* Cruise duration */
    float t_cruise = (d - 2.0f * d_acc) / v_peak;
    if (t_cruise < 0.0f) t_cruise = 0.0f;

    /* Cumulative segment boundary times */
    sc->T[0] = 0.0f;
    sc->T[1] = t1_used;                      /* end seg 1: jerk up   */
    sc->T[2] = sc->T[1] + t2;               /* end seg 2: const a   */
    sc->T[3] = sc->T[2] + t1_used;          /* end seg 3: jerk down */
    sc->T[4] = sc->T[3] + t_cruise;         /* end seg 4: cruise    */
    sc->T[5] = sc->T[4] + t1_used;          /* end seg 5: jerk down */
    sc->T[6] = sc->T[5] + t2;               /* end seg 6: const -a  */
    sc->T[7] = sc->T[6] + t1_used;          /* end seg 7: jerk up   */
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

    /* Back-compute profile from segment times.
     *
     * Area identity (proven in docs):  d = v_peak × (t_acc + t_cruise)
     *   where t_acc = 2*t1 + t2
     *
     * Then:  a_peak = v_peak / (t1 + t2)
     *        j_used = a_peak / t1
     */
    float t_acc  = 2.0f * t1 + t2;
    float v_peak = d / (t_acc + t_cruise);
    float a_peak = (t1 + t2 > 1e-6f) ? v_peak / (t1 + t2) : v_peak / t1;

    sc->v_peak = v_peak;
    sc->a_peak = a_peak;

    /* Segment boundary times */
    sc->T[0] = 0.0f;
    sc->T[1] = t1;
    sc->T[2] = t1 + t2;
    sc->T[3] = t1 + t2 + t1;
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
