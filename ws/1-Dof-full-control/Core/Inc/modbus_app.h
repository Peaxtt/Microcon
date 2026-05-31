#ifndef __MODBUS_APP_H
#define __MODBUS_APP_H

#include "modbus.h"
#include "robot.h"

/* =========================================================================
 * modbus_app.h — Modbus Application Layer
 *
 * Translates Modbus registers ↔ robot state.
 * Two functions called every 100Hz from the main loop:
 *
 *   modbus_app_receive() — registers → commands (before state_machine_tick)
 *   modbus_app_send()    — robot state → registers (after state_machine_tick)
 *
 * Register map (Slave ID = 21, LPUART1, 19200 8E1):
 *
 * WRITE — BaseSystem → MCU (modbus_app_receive handles these)
 * ┌────────┬──────────────────────────────────────────────────────────────┐
 * │ 0x00   │ Heartbeat: write 18537 (HI) → MCU replies 22881 (YA)        │
 * │ 0x01   │ Mode command (bit-field, auto-clears):                       │
 * │        │   0x01=HOME  0x02=MANUAL  0x04=AUTO  0x08=SET_HOME          │
 * │        │   0x10=TEST (requires homed + in AUTO)                       │
 * │ 0x02   │ Actuator one-hot: 0x01=cylinder 0x02=grip-open 0x04=grip-close│
 * │ 0x03   │ Manual sequence step: bit0=pick bit1=place                   │
 * │ 0x04   │ Sequence autostart: bit0=1 → run loaded pairs               │
 * │ 0x05   │ Jog offset (signed degrees, auto-clears)                     │
 * │ 0x06   │ Test params apply: bit0=1 → apply 0x07-0x0B                 │
 * │ 0x07   │ Test v_max (deg/s)                                           │
 * │ 0x08   │ Test a_max (deg/s²)                                          │
 * │ 0x09   │ Test start position (deg)                                    │
 * │ 0x10   │ Test end position (deg)                                      │
 * │ 0x11   │ Test repeat count                                            │
 * │ 0x12+  │ Sequence pairs: [0x12+i*2]=pick_deg [0x13+i*2]=place_deg    │
 * │ 0x15   │ Move command: write 1 → apply target from 0x0B + ctrl params│
 * │ 0x16   │ Set Home: write 1 → virtual home at current position         │
 * │ 0x19   │ Soft STOP: write 1 → hold current position                  │
 * │ 0x22   │ Sequence pair count                                          │
 * │ 0x24   │ P2P target (degrees, edge-triggered)                         │
 * │ 0x25   │ Pause: bit0=1 → stop trajectory, hold position              │
 * │ 0x33   │ System mode (0=PROD 1=HW_TEST 2=JOY_TEST 3=AUTO_MOTOR)      │
 * │ 0x34   │ max_speed ×100                                               │
 * │ 0x35   │ ramp_rate ×1000 (unused, stored for ctrl)                    │
 * │ 0x36   │ encoder_inverted — READ ONLY from Modbus (Live Exp only)     │
 * │ 0x38   │ traj_type (0=trap 1=scurve 2=direct)                        │
 * │ 0x39   │ v_max ×100 (rad/s)                                           │
 * │ 0x3B   │ a_max ×100 (rad/s²)                                          │
 * │ 0x3C   │ j_max ×100 (rad/s³)                                          │
 * │ 0x3D   │ kp_pos ×100                                                  │
 * │ 0x3E   │ kd_pos ×100                                                  │
 * │ 0x3F   │ ki_pos ×100                                                  │
 * │ 0x0B   │ Move target (rad × 1000, int16)                              │
 * │ 0x0C   │ kp_vel ×100                                                  │
 * │ 0x0D   │ ki_vel ×100                                                  │
 * │ 0x0E   │ kd_vel ×100                                                  │
 * └────────┴──────────────────────────────────────────────────────────────┘
 *
 * READ — MCU → BaseSystem (modbus_app_send writes these every 100Hz)
 * ┌────────┬──────────────────────────────────────────────────────────────┐
 * │ 0x00   │ Heartbeat reply: 22881 (YA)                                  │
 * │ 0x23   │ Homed flag (0=not homed 1=homed)                             │
 * │ 0x26   │ Reed switches: bit0=up bit1=down bit2=grip                   │
 * │ 0x27   │ Status flags: 0x01=homing 0x02=picking 0x04=placing         │
 * │        │               0x08=moving  0x10=sequence  0x20=test          │
 * │ 0x28   │ Position 0–359.99° × 10 (int16)                              │
 * │ 0x29   │ Velocity deg/s × 10 (int16)                                  │
 * │ 0x2F   │ Current state (RobotState_t enum 0–10)                        │
 * │ 0x30   │ Acceleration deg/s² × 10 (int16)                             │
 * │ 0x31   │ ESTOP active (1 = emergency)                                  │
 * │ 0x32   │ Digital IO: bit0=HOME_SENSOR bit1=POWER_BTN                  │
 * │        │             bit2=RESET_BTN   bit3=MODE_SW                    │
 * │ 0x3A   │ Current sensor mA (uint16)                                   │
 * └────────┴──────────────────────────────────────────────────────────────┘
 * ========================================================================= */

/* Call before state_machine_tick: decode registers into robot commands */
void modbus_app_receive(void);

/* Call after state_machine_tick: encode robot state into feedback registers */
void modbus_app_send(void);

/* External: Modbus slave instance (defined in main.c) */
extern ModbusSlave_t mb_slave;

#endif /* __MODBUS_APP_H */
