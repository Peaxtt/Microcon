/*
 * DistFF.c
 *
 * Disturbance Feedforward Filter  —  1st-order Tustin-discretised.
 *
 * Difference equation:
 *   y[k] = ( b0·u[k] + b1·u[k-1] - a1·y[k-1] ) / a0
 *
 *   u[k] = tau_d_hat  (Kalman4 disturbance estimate, N·m)
 *   y[k] = V_ff_d     (feedforward voltage, V)
 */

#include "DistFF.h"

void DistFF_Init(DistFF_t *ff,
                 float L, float R,
                 float N, float Kt,
                 float tau, float Ts)
{
    ff->L   = L;
    ff->R   = R;
    ff->N   = N;
    ff->Kt  = Kt;
    ff->tau = tau;
    ff->Ts  = Ts;

    /* Continuous coefficients
     *   n1 = L,          n0 = R
     *   d1 = N·Kt·tau,   d0 = N·Kt
     */
    float n1 = L;
    float n0 = R;
    float d1 = N * Kt * tau;
    float d0 = N * Kt;

    /* Tustin pre-warping constant  c = 2 / Ts */
    float c = 2.0f / Ts;

    /* Discrete coefficients */
    ff->b0 =  n1 * c + n0;
    ff->b1 = -n1 * c + n0;

    ff->a0 =  d1 * c + d0;
    ff->a1 = -d1 * c + d0;

    DistFF_Reset(ff);
}

void DistFF_Reset(DistFF_t *ff)
{
    ff->u1 = 0.0f;
    ff->y1 = 0.0f;
}

float DistFF_Update(DistFF_t *ff, float tau_d_hat)
{
    float y = (ff->b0 * tau_d_hat
             + ff->b1 * ff->u1
             - ff->a1 * ff->y1) / ff->a0;

    ff->u1 = tau_d_hat;
    ff->y1 = y;

    return y;
}
