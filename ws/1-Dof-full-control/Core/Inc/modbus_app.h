#ifndef MODBUS_APP_H
#define MODBUS_APP_H

#include "robot.h"

/* ── Modbus Register Map Constants ───────────────────────────────────────── */
#define MB_REG_HEARTBEAT    0x00  /* YA/HI handshake */
#define MB_REG_MODE_CMD     0x01  /* mode command (0=idle,1=home,2=manual,4=auto,8=sethome,16=test) */
#define MB_REG_ACT_CMD      0x03  /* actuator command bit0=pick bit1=place */
#define MB_REG_SEQ_START    0x04  /* sequence autostart */
#define MB_REG_JOG_DEG      0x05  /* jog relative move (degrees × 10, signed) */
#define MB_REG_SEQ_COUNT    0x06  /* number of pick/place pairs */
#define MB_REG_SEQ_PAIR0    0x07  /* pick/place pair 0 pick-hole (base of pair array) */

#define MB_REG_PYTHON_STEP  10    /* Python step cmd */
#define MB_REG_PYTHON_POS   11    /* Python target pos (rad × 1000) */
#define MB_REG_PYTHON_KP    12
#define MB_REG_PYTHON_KI    13
#define MB_REG_PYTHON_KD    14
#define MB_REG_PYTHON_APPLY 15
#define MB_REG_SET_HOME     21    /* reg[21]=1 → set home */
#define MB_REG_P2P_TARGET   0x24  /* P2P target (degrees, signed) */
#define MB_REG_STOP         0x25  /* reg[25]=1 → soft stop */
#define MB_REG_HOMING_DONE  0x27  /* homing done flag */

#define MB_REG_HOMED        0x23
#define MB_REG_P2P_RESULT   0x24  /* also read: last P2P target echo */
#define MB_REG_REED         0x26  /* bit0=up bit1=down bit2=grip */
#define MB_REG_POS          0x28  /* position deg×10 */
#define MB_REG_VEL          0x29  /* velocity deg/s×10 */
#define MB_REG_STATE        0x2F
#define MB_REG_ACC          0x30
#define MB_REG_ESTOP        0x31
#define MB_REG_DIG_IO       0x32
#define MB_REG_SYS_MODE     0x33
#define MB_REG_MAX_SPEED    0x34
#define MB_REG_RAMP_RATE    0x35
#define MB_REG_ENC_INV      0x36
#define MB_REG_TRAJ_TYPE    0x38
#define MB_REG_VMAX         0x39
#define MB_REG_CURRENT_MA   0x3A
#define MB_REG_AMAX         0x3B
#define MB_REG_JMAX         0x3C
#define MB_REG_KP_POS       0x3D
#define MB_REG_KD_POS       0x3E
#define MB_REG_KI_POS       0x3F

/* ── API ─────────────────────────────────────────────────────────────────── */
void modbus_app_receive(void);
void modbus_app_send(void);

#endif /* MODBUS_APP_H */
