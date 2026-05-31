#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "robot.h"
#include "hardware.h"
#include "control.h"
#include "joystick.h"

/* =========================================================================
 * state_machine.h — Robot State Machine
 *
 * Runs at 100 Hz. Reads commands from robot.h, drives outputs via hardware.h.
 * One public function called from the main loop.
 * ========================================================================= */

/* Call every 10ms from main loop (after modbus_app_receive, before modbus_app_send) */
void state_machine_tick(void);

#endif /* __STATE_MACHINE_H */
