#include "REF_FEEDFORWARD.h"

void RefFF_Init(RefFF_t *ff,
                float J_eq, float L, float R, float b_eq,
                float N,    float Kt, float Ke,
                float tau,  float Ts)
{
    ff->J_eq = J_eq;
    ff->L    = L;
    ff->R    = R;
    ff->b_eq = b_eq;
    ff->N    = N;
    ff->Kt   = Kt;
    ff->Ke   = Ke;
    ff->tau  = tau;
    ff->Ts   = Ts;

    /* Continuous numerator coefficients */
    float n2 = J_eq * L;
    float n1 = J_eq * R  + b_eq * L;
    float n0 = b_eq * R  + N * N * Kt * Ke;

    /* Continuous denominator coefficients */
    float d2 = N * Kt * tau * tau;
    float d1 = 2.0f * N * Kt * tau;
    float d0 = N * Kt;

    /* Tustin pre-warping constant  c = 2 / Ts */
    float c  = 2.0f / Ts;
    float c2 = c * c;

    /* Discrete numerator */
    ff->b0 =  n2 * c2 + n1 * c + n0;
    ff->b1 = -2.0f * n2 * c2   + 2.0f * n0;
    ff->b2 =  n2 * c2 - n1 * c + n0;

    /* Discrete denominator */
    ff->a0 =  d2 * c2 + d1 * c + d0;
    ff->a1 = -2.0f * d2 * c2   + 2.0f * d0;
    ff->a2 =  d2 * c2 - d1 * c + d0;

    RefFF_Reset(ff);
}

void RefFF_Reset(RefFF_t *ff)
{
    ff->u1 = 0.0f;
    ff->u2 = 0.0f;
    ff->y1 = 0.0f;
    ff->y2 = 0.0f;
}

float RefFF_Update(RefFF_t *ff, float omega_ref)
{
    float y = (  ff->b0 * omega_ref
               + ff->b1 * ff->u1
               + ff->b2 * ff->u2
               - ff->a1 * ff->y1
               - ff->a2 * ff->y2 ) / ff->a0;

    ff->u2 = ff->u1;  ff->u1 = omega_ref;
    ff->y2 = ff->y1;  ff->y1 = y;

    return y;
}
