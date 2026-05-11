#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float v_max;
    float a_max;
    float dt;

    float v_peak;
    float T1;
    float T2;
    float T3;
    float dir;

    float t_now;
    float v_current;
    float a_current;
    float p_current;

    uint8_t is_active;
} Trapezoid_t;

void Trapezoid_Init(Trapezoid_t *tr, float v_max, float a_max, float dt);
void Trapezoid_SetTarget(Trapezoid_t *tr, float displacement);
void Trapezoid_SetTarget_ByTime(Trapezoid_t *tr, float displacement, float t_acc, float t_cruise);
void Trapezoid_Update(Trapezoid_t *tr);

#endif
