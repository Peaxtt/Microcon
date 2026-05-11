#ifndef SCURVE_H
#define SCURVE_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float v_max;
    float a_max;
    float j_max;
    float dt;

    float T[8];
    float v_peak;
    float a_peak;
    float dir;

    float t_now;
    float v_current;
    float a_current;
    float j_current;
    float p_current;

    uint8_t is_active;
} SCurve_t;

void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max, float dt);
void SCurve_SetTarget(SCurve_t *sc, float displacement);
void SCurve_SetTarget_ByTime(SCurve_t *sc, float displacement, float t1, float t2, float t_cruise);
void SCurve_Update(SCurve_t *sc);

#endif
