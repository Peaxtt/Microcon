/*
 * control_defaults.c — Weak default implementations for the control layer.
 *
 * These stubs allow the firmware to build and run without control.c.
 * When control.c is linked (by the control team), these are overridden.
 *
 * Default behaviour: motor stays off, control_is_settled always returns 1
 * so the state machine can still transition through states for testing.
 */
#include "control.h"

__weak void    control_init       (void)                                    { }
__weak void    control_set_target (float target_rad)                        { (void)target_rad; }
__weak void    control_update     (float pos, float vel, float *pwm_out)    { (void)pos; (void)vel; *pwm_out = 0.0f; }
__weak void    control_reset      (void)                                    { }
__weak uint8_t control_is_settled (float pos_rad)                          { (void)pos_rad; return 1; }
