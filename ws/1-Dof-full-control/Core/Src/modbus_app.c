#include "modbus_app.h"
#include <math.h>

/* Heartbeat tokens */
#define HB_PING  18537u   /* BaseSystem writes this */
#define HB_PONG  22881u   /* MCU replies with this  */

/* ── modbus_app_receive ──────────────────────────────────────────────────── */
/* Decode incoming registers into robot commands.
 * Mode commands (0x01) are gated by cabinet switch (robot_mode_sw):
 *   switch LOW  (AUTO)  → full access
 *   switch HIGH (MANUAL)→ only HOME and SET_HOME allowed */
void modbus_app_receive(void)
{
  uint16_t *r = mb_slave.registers;   /* shorthand */
  uint8_t sw_auto = (robot_mode_sw == 0); /* LOW = AUTO position */

  /* ── 0x00: Heartbeat ─────────────────────────────────────────────────── */
  /* Handled in modbus_app_send */

  /* ── 0x01: Mode command (bit-field, always auto-clears) ─────────────── */
  if (r[0x01]) {
    uint16_t cmd = r[0x01];
    r[0x01] = 0;   /* always clear, even if blocked */

    if (robot_state != ST_EMER &&
        robot_state != ST_HOMING_FAST &&
        robot_state != ST_HOMING_BACKOFF &&
        robot_state != ST_HOMING_SLOW) {

      if (cmd & 0x01) {                                /* HOME — always allowed */
        cmd_do_home = 1;
      }
      if ((cmd & 0x02) && sw_auto) {                  /* MANUAL — switch gate  */
        cmd_go_manual = 1;
      }
      if ((cmd & 0x04) && sw_auto && robot_homed) {   /* AUTO — switch + homed */
        cmd_go_auto = 1;
      }
      if (cmd & 0x08) {                                /* SET HOME              */
        cmd_set_home = 1;
      }
      if ((cmd & 0x10) && sw_auto && robot_homed &&
          robot_state == ST_AUTO) {                    /* TEST — in AUTO + homed */
        cmd_go_test = 1;
      }
    }
  }

  /* ── 0x02: Actuator one-hot ─────────────────────────────────────────── */
  if (r[0x02]) {
    cmd_actuator = (uint8_t)r[0x02];
    r[0x02] = 0;
  }

  /* ── 0x03: Manual sequence step ─────────────────────────────────────── */
  if (r[0x03] & 0x01) { cmd_seq_pick  = 1; r[0x03] &= ~0x01; }
  if (r[0x03] & 0x02) { cmd_seq_place = 1; r[0x03] &= ~0x02; }

  /* ── 0x04: Sequence autostart ───────────────────────────────────────── */
  if (r[0x04] & 0x01) {
    cmd_seq_start = 1;
    r[0x04] = 0;
  }

  /* ── 0x05: Jog (signed degrees, wrap 0-360) ─────────────────────────── */
  if (r[0x05] && sw_auto &&
      (robot_state == ST_AUTO || robot_state == ST_MANUAL_MODBUS)) {
    cmd_jog_deg = (float)(int16_t)r[0x05];
    r[0x05] = 0;
  }

  /* ── 0x06: Test param apply ─────────────────────────────────────────── */
  if (r[0x06] & 0x01) {
    /* Store test params into control model — ctrl will use on next move */
    if (r[0x07] > 0) control_model[0].v_max = (float)r[0x07] * (3.14159265f / 180.0f);
    if (r[0x08] > 0) control_model[0].a_max = (float)r[0x08] * (3.14159265f / 180.0f);
    r[0x06] = 0;
  }

  /* ── 0x0B/0x15: Direct move command ─────────────────────────────────── */
  if (r[0x15] == 1 && sw_auto) {
    float tgt_rad = (float)(int16_t)r[0x0B] / 1000.0f;
    robot_target_rad = tgt_rad;
    robot_move_armed = 1;
    if (robot_state != ST_EMER) robot_state = ST_AUTO;
    r[0x15] = 0;
  }

  /* ── 0x16: Set Home ─────────────────────────────────────────────────── */
  if (r[0x16] == 1) {
    cmd_set_home = 1;
    r[0x16] = 0;
  }

  /* ── 0x19: Soft STOP ─────────────────────────────────────────────────── */
  if (r[0x19] == 1) {
    cmd_soft_stop = 1;
    r[0x19] = 0;
  }

  /* ── 0x22 + 0x12+: Sequence pairs ──────────────────────────────────── */
  {
    uint8_t count = (uint8_t)r[0x22];
    if (count > ROBOT_SEQ_MAX) count = ROBOT_SEQ_MAX;
    seq_count = count;
    for (uint8_t i = 0; i < count; i++) {
      seq_pick_deg[i]  = (float)(int16_t)r[0x12 + i * 2];
      seq_place_deg[i] = (float)(int16_t)r[0x13 + i * 2];
    }
  }

  /* ── 0x24: P2P target (edge-triggered) ──────────────────────────────── */
  if (sw_auto && (robot_state == ST_AUTO || robot_state == ST_MANUAL_MODBUS)) {
    float p2p = (float)(int16_t)r[0x24];
    if (p2p != cmd_p2p_last) {
      cmd_p2p_last = p2p;
      cmd_p2p_deg  = p2p;
    }
  }

  /* ── 0x25: Pause ─────────────────────────────────────────────────────── */
  cmd_pause = (r[0x25] & 0x01) ? 1 : 0;

  /* ── 0x34: max_speed ×100 ───────────────────────────────────────────── */
  if (r[0x34] >= 1 && r[0x34] <= 100)
    sys_max_speed = (float)r[0x34] / 100.0f;
  else
    r[0x34] = (uint16_t)(sys_max_speed * 100.0f);

  /* ── 0x36: encoder_inverted — read-only mirror ──────────────────────── */
  r[0x36] = (uint16_t)encoder_inverted;

  /* ── 0x38–0x3F: Trajectory / position PID params (store for ctrl) ───── */
  if (r[0x38] <= 2) control_model[0].traj_type = (uint8_t)r[0x38];
  if (r[0x39] > 0)  control_model[0].v_max     = (float)r[0x39] / 100.0f;
  if (r[0x3B] > 0)  control_model[0].a_max     = (float)r[0x3B] / 100.0f;
  if (r[0x3C] > 0)  control_model[0].j_max     = (float)r[0x3C] / 100.0f;
  if (r[0x3D] > 0)  control_model[0].kp_pos    = (float)r[0x3D] / 100.0f;
  control_model[0].kd_pos = (float)r[0x3E] / 100.0f;
  control_model[0].ki_pos = (float)r[0x3F] / 100.0f;

  /* ── 0x0C–0x0E: Velocity PID gains (store for ctrl) ─────────────────── */
  if (r[0x0C] > 0) control_model[0].kp_vel = (float)r[0x0C] / 100.0f;
  control_model[0].ki_vel = (float)r[0x0D] / 100.0f;
  control_model[0].kd_vel = (float)r[0x0E] / 100.0f;
}

/* ── modbus_app_send ─────────────────────────────────────────────────────── */
/* Write robot state into feedback registers. Called every 100Hz. */
void modbus_app_send(void)
{
  uint16_t *r = mb_slave.registers;

  /* ── 0x00: Heartbeat ─────────────────────────────────────────────────── */
  static uint32_t hb_timer = 0;
  if (r[0x00] == HB_PING) {
    hb_timer = 0;
    r[0x00]  = HB_PONG;
  } else {
    hb_timer++;
  }
  (void)hb_timer; /* watchdog: future use */

  /* ── 0x2F: Current state ─────────────────────────────────────────────── */
  r[0x2F] = (uint16_t)robot_state;

  /* ── 0x23: Homed flag ────────────────────────────────────────────────── */
  r[0x23] = (uint16_t)robot_homed;

  /* ── 0x28: Position 0–359.99° × 10 ──────────────────────────────────── */
  r[0x28] = (uint16_t)(int16_t)(robot_pos_deg * 10.0f);

  /* ── 0x29: Velocity deg/s × 10 ──────────────────────────────────────── */
  r[0x29] = (uint16_t)(int16_t)(robot_vel_rad_s * (180.0f / 3.14159265f) * 10.0f);

  /* ── 0x30: Acceleration deg/s² × 10 ─────────────────────────────────── */
  r[0x30] = (uint16_t)(int16_t)(robot_acc_rad_s2 * (180.0f / 3.14159265f) * 10.0f);

  /* ── 0x26: Reed switches ─────────────────────────────────────────────── */
  r[0x26] = (robot_reed_up   ? 0x01 : 0) |
            (robot_reed_down ? 0x02 : 0) |
            (robot_reed_grip ? 0x04 : 0);

  /* ── 0x27: Status flags ──────────────────────────────────────────────── */
  {
    uint16_t flags = 0;
    if (robot_state >= ST_HOMING_FAST && robot_state <= ST_HOMING_SLOW) flags |= 0x01;
    if (robot_state == ST_SEQUENCE)  flags |= 0x10;
    if (robot_state == ST_TEST)      flags |= 0x20;
    if (robot_move_armed)            flags |= 0x08;
    r[0x27] = flags;
  }

  /* ── 0x31: ESTOP active ──────────────────────────────────────────────── */
  r[0x31] = (uint16_t)robot_estop;

  /* ── 0x32: Digital IO bitfield ──────────────────────────────────────── */
  r[0x32] = (robot_home_sensor ? 0x01 : 0) |
            (robot_power_btn   ? 0x02 : 0) |
            (robot_reset_btn   ? 0x04 : 0) |
            (robot_mode_sw     ? 0x08 : 0);

  /* ── 0x3A: Current sensor mA ─────────────────────────────────────────── */
  r[0x3A] = (uint16_t)(robot_current_A * 1000.0f);

  /* ── Control params echo (so BaseSystem can read back what was set) ─── */
  r[0x38] = (uint16_t)control_model[0].traj_type;
  r[0x39] = (uint16_t)(control_model[0].v_max * 100.0f);
  r[0x3B] = (uint16_t)(control_model[0].a_max * 100.0f);
  r[0x3C] = (uint16_t)(control_model[0].j_max * 100.0f);
  r[0x3D] = (uint16_t)(control_model[0].kp_pos * 100.0f);
  r[0x3E] = (uint16_t)(control_model[0].kd_pos * 100.0f);
  r[0x3F] = (uint16_t)(control_model[0].ki_pos * 100.0f);
  r[0x0C] = (uint16_t)(control_model[0].kp_vel * 100.0f);
  r[0x0D] = (uint16_t)(control_model[0].ki_vel * 100.0f);
  r[0x0E] = (uint16_t)(control_model[0].kd_vel * 100.0f);
}
