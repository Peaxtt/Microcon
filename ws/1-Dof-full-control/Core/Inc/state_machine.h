#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "robot.h"

void state_machine_tick(void);

/* homing helpers — also called from main.c startup */
void homing_exti_enable(void);
void homing_exti_disable(void);
void finish_homing(void);

#endif /* STATE_MACHINE_H */
