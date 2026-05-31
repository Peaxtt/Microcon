#include "state_machine.h"
#include <math.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Convert degrees to radians, wrapping result to 0–2π if homed */
static float deg_to_rad_wrap(float deg)
{
  float rad = deg * (3.14159265f / 180.0f);
  return rad;
}

/* Arm a new move: set target, notify ISR, enter AUTO */
static void do_move_deg(float deg)
{
  /* Wrap to 0–359.99° */
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  robot_target_rad = deg_to_rad_wrap(deg);
  robot_move_armed = 1;
  control_set_target(robot_target_rad);
}

/* ── Tower light logic (called every tick) ──────────────────────────────── */
static void update_tower_lights(uint8_t tick)
{
  uint8_t r = 0, y = 0, g = 0;

  switch (robot_state) {

    case ST_EMER:
      if (!robot_estop) {
        /* ESTOP released: solid red, waiting for RESET */
        r = 1;
      } else {
        /* ESTOP still pressed: red blink 2Hz */
        r = (tick % 50) < 25 ? 1 : 0;
      }
      break;

    case ST_HOMING_FAST:
    case ST_HOMING_BACKOFF:
    case ST_HOMING_SLOW:
      /* Yellow fast blink 5Hz during homing */
      y = (tick % 20) < 10 ? 1 : 0;
      break;

    case ST_IDLE:
      if (!robot_homed) {
        y = 1;          /* Not homed: yellow steady */
      } else {
        g = 1;          /* Homed and idle: green steady */
      }
      break;

    case ST_MANUAL_SWITCH:
    case ST_MANUAL_MODBUS:
      y = 1;            /* Manual: yellow steady */
      break;

    case ST_AUTO:
      /* Green steady when idle, slow blink when moving */
      g = robot_move_armed ? (tick % 100) < 50 ? 1 : 0 : 1;
      break;

    case ST_SEQUENCE:
      y = 1; g = 1;    /* Sequence: yellow + green */
      break;

    case ST_TEST:
      /* Green + yellow blink 2Hz */
      g = 1;
      y = (tick % 50) < 25 ? 1 : 0;
      break;

    default:
      break;
  }

  /* Near-limit warning: red brief flash when within 80° of ±720° */
  if (robot_state != ST_EMER && robot_homed &&
      fabsf(robot_cumul_deg) > 440.0f) {
    r = (tick % 20) < 3 ? 1 : 0;
  }

  hardware_set_tower(r, y, g);
}

/* ── Actuator helper ─────────────────────────────────────────────────────── */
static void handle_actuator_cmd(void)
{
  if (!cmd_actuator) return;
  uint8_t cyl        = (cmd_actuator & 0x01) ? 1 : 0;
  uint8_t grip_open  = (cmd_actuator & 0x02) ? 1 : 0;
  uint8_t grip_close = (cmd_actuator & 0x04) ? 1 : 0;
  hardware_set_actuator(cyl, grip_open, grip_close);
  cmd_actuator = 0;
}

/* ── Sequence state machine (nested inside ST_SEQUENCE) ─────────────────── */
typedef enum {
  SEQ_IDLE = 0,
  SEQ_GOING_PICK,
  SEQ_PICKING,
  SEQ_GOING_PLACE,
  SEQ_PLACING,
} SeqState_t;

static SeqState_t seq_state     = SEQ_IDLE;
static uint8_t    seq_pair_idx  = 0;
static uint32_t   seq_delay_cnt = 0;

static void sequence_tick(void)
{
  /* Delay in 10ms ticks */
  uint32_t grip_delay_ticks = (uint32_t)(reed_dummy_delay_ms / 10.0f);

  switch (seq_state) {

    case SEQ_IDLE:
      if (seq_pair_idx < seq_count) {
        do_move_deg(seq_pick_deg[seq_pair_idx]);
        seq_state = SEQ_GOING_PICK;
      } else {
        /* All pairs done — return to AUTO */
        seq_state    = SEQ_IDLE;
        seq_pair_idx = 0;
        robot_state  = ST_AUTO;
      }
      break;

    case SEQ_GOING_PICK:
      if (control_is_settled(robot_pos_rad)) {
        /* Arrived at pick → lower cylinder, wait for reed or dummy delay */
        hardware_set_actuator(1, 0, 0); /* extend cylinder */
        seq_delay_cnt = 0;
        seq_state     = SEQ_PICKING;
      }
      break;

    case SEQ_PICKING:
      seq_delay_cnt++;
      if (robot_reed_down || seq_delay_cnt >= grip_delay_ticks) {
        hardware_set_actuator(1, 0, 1); /* close gripper */
        seq_delay_cnt = 0;
        do_move_deg(seq_place_deg[seq_pair_idx]);
        seq_state = SEQ_GOING_PLACE;
      }
      break;

    case SEQ_GOING_PLACE:
      if (control_is_settled(robot_pos_rad)) {
        hardware_set_actuator(1, 1, 0); /* open gripper → release */
        seq_delay_cnt = 0;
        seq_state     = SEQ_PLACING;
      }
      break;

    case SEQ_PLACING:
      seq_delay_cnt++;
      if (seq_delay_cnt >= grip_delay_ticks) {
        hardware_set_actuator(0, 0, 0); /* retract, gripper neutral */
        seq_pair_idx++;
        seq_state = SEQ_IDLE;          /* → next pair */
      }
      break;
  }
}

/* ── EMER entry / exit state ─────────────────────────────────────────────── */
static uint8_t  emer_debounce   = 0;   /* ESTOP must be LOW for 20 ticks */
static uint16_t reset_hold_cnt  = 0;   /* RESET must be held for 5 ticks */

/* ── main state_machine_tick ─────────────────────────────────────────────── */
void state_machine_tick(void)
{
  static uint8_t tick = 0;
  tick++;   /* wraps at 255, used for light blinking */

  /* ════════════════════════════════════════════════════════════════════════
   * LEVEL 1: EMER — wins over everything
   * ════════════════════════════════════════════════════════════════════════ */

  /* ESTOP debounce: must be active for 20 consecutive ticks (200ms) */
  if (robot_estop) {
    if (++emer_debounce >= 20 && robot_state != ST_EMER) {
      robot_state  = ST_EMER;
      hardware_set_motor(0.0f);
      hardware_set_emer_output(1);
      control_reset();
      seq_state    = SEQ_IDLE;
      seq_pair_idx = 0;
    }
  } else {
    emer_debounce = 0;
  }

  /* Joystick LB → EMER (any state) */
  if (joy_is_connected() && joy_btn(BTN_LB) && robot_state != ST_EMER) {
    robot_state = ST_EMER;
    hardware_set_motor(0.0f);
    hardware_set_emer_output(1);
    control_reset();
    seq_state    = SEQ_IDLE;
    seq_pair_idx = 0;
  }

  /* ── EMER exit: ESTOP released + RESET held 50ms ─────────────────────── */
  if (robot_state == ST_EMER) {
    if (!robot_estop && robot_reset_btn) {
      if (++reset_hold_cnt >= 5) {  /* 5 × 10ms = 50ms */
        reset_hold_cnt = 0;
        hardware_set_emer_output(0);
        hardware_set_motor(0.0f);
        robot_state = ST_IDLE;
        /* Clear any leftover commands */
        cmd_do_home = cmd_go_manual = cmd_go_auto = cmd_go_test = 0;
        cmd_soft_stop = 0; cmd_jog_deg = 0.0f;
        cmd_p2p_last = -9999.0f;
      }
    } else {
      reset_hold_cnt = 0;
    }
    hardware_set_reset_led(robot_reset_btn && !robot_estop ? 1 : 0);
    update_tower_lights(tick);
    return;  /* nothing else runs during EMER */
  }

  /* ════════════════════════════════════════════════════════════════════════
   * LEVEL 2: Cabinet switch HIGH → ST_MANUAL_SWITCH (joystick motor only)
   * ════════════════════════════════════════════════════════════════════════ */

  if (robot_mode_sw) {  /* HIGH = MANUAL */
    if (robot_state != ST_MANUAL_SWITCH) {
      /* Just flipped to MANUAL — stop motor, enter MANUAL_SWITCH */
      hardware_set_motor(0.0f);
      control_reset();
      robot_state  = ST_MANUAL_SWITCH;
      seq_state    = SEQ_IDLE;
    }
  } else {
    /* Just flipped to AUTO — go to IDLE */
    if (robot_state == ST_MANUAL_SWITCH) {
      hardware_set_motor(0.0f);
      robot_state = ST_IDLE;
    }
  }

  /* ════════════════════════════════════════════════════════════════════════
   * LEVEL 3: Per-state logic
   * ════════════════════════════════════════════════════════════════════════ */

  switch (robot_state) {

    /* ── INIT ──────────────────────────────────────────────────────────── */
    case ST_INIT:
      hardware_set_motor(0.0f);
      robot_state = ST_IDLE;
      break;

    /* ── IDLE ──────────────────────────────────────────────────────────── */
    case ST_IDLE:
      hardware_set_motor(0.0f);
      /* Accept mode commands */
      if (cmd_do_home)    { cmd_do_home = 0; hardware_set_motor(0.0f); control_reset(); hardware_homing_sensor_enable(); robot_state = ST_HOMING_FAST; }
      else if (cmd_go_manual) { cmd_go_manual = 0; control_reset(); robot_state = ST_MANUAL_MODBUS; }
      else if (cmd_go_auto && robot_homed) { cmd_go_auto = 0; control_reset(); cmd_p2p_last = cmd_p2p_deg; robot_state = ST_AUTO; }
      if (cmd_set_home)   { cmd_set_home = 0; hardware_finish_homing(); }
      handle_actuator_cmd();
      break;

    /* ── HOMING FAST: direct PWM left until sensor ─────────────────────── */
    case ST_HOMING_FAST:
      hardware_set_motor(-homing_fast_pct);
      if (robot_home_sensor) {
        robot_home_sensor = 0;
        hardware_set_motor(0.0f);
        hardware_homing_sensor_disable();
        robot_state = ST_HOMING_BACKOFF;
      }
      break;

    /* ── HOMING BACKOFF: direct PWM right for N ticks ──────────────────── */
    case ST_HOMING_BACKOFF: {
      static uint32_t back_tick = 0;
      hardware_set_motor(homing_backoff_pct);
      if (++back_tick >= homing_backoff_ticks) {
        back_tick = 0;
        hardware_set_motor(0.0f);
        hardware_homing_sensor_enable();
        robot_state = ST_HOMING_SLOW;
      }
      break;
    }

    /* ── HOMING SLOW: direct PWM left until sensor (final pass) ────────── */
    case ST_HOMING_SLOW:
      hardware_set_motor(-homing_slow_pct);
      if (robot_home_sensor) {
        robot_home_sensor = 0;
        hardware_set_motor(0.0f);
        hardware_finish_homing();
        /* hardware_finish_homing arms home offset move if set */
        if (robot_move_armed)
          robot_state = ST_AUTO;
        else
          robot_state = ST_IDLE;
      }
      break;

    /* ── MANUAL_SWITCH: joystick controls motor ─────────────────────────── */
    case ST_MANUAL_SWITCH: {
      float pct = 0.0f;
      if (joy_is_connected()) {
        pct = joy_ly_f();     /* left stick Y: -1..+1 */
        /* Apply dead zone */
        if (fabsf(pct) < 0.08f) pct = 0.0f;
      }
      hardware_set_motor(pct * sys_max_speed);
      handle_actuator_cmd();
      break;
    }

    /* ── MANUAL_MODBUS: Modbus jog + actuators ──────────────────────────── */
    case ST_MANUAL_MODBUS:
      hardware_set_motor(0.0f);
      if (cmd_do_home) { cmd_do_home = 0; control_reset(); hardware_homing_sensor_enable(); robot_state = ST_HOMING_FAST; break; }
      if (cmd_go_auto && robot_homed) { cmd_go_auto = 0; control_reset(); robot_state = ST_AUTO; break; }

      /* Jog: relative offset, wraps 0-360° */
      if (cmd_jog_deg != 0.0f) {
        float now_deg = robot_pos_deg;
        float tgt     = now_deg + cmd_jog_deg;
        tgt = fmodf(tgt, 360.0f);
        if (tgt < 0.0f) tgt += 360.0f;
        cmd_jog_deg = 0.0f;
        do_move_deg(tgt);
        robot_state = ST_AUTO;  /* let AUTO track the jog */
      }
      handle_actuator_cmd();
      break;

    /* ── AUTO: P2P moves from Modbus ────────────────────────────────────── */
    case ST_AUTO: {
      /* Soft stop */
      if (cmd_soft_stop) {
        cmd_soft_stop = 0;
        robot_move_armed = 0;
        control_reset();
        hardware_set_motor(0.0f);
        break;
      }
      /* Mode exits */
      if (cmd_do_home)    { cmd_do_home = 0; control_reset(); hardware_set_motor(0.0f); hardware_homing_sensor_enable(); robot_state = ST_HOMING_FAST; break; }
      if (cmd_go_manual)  { cmd_go_manual = 0; control_reset(); hardware_set_motor(0.0f); robot_state = ST_MANUAL_MODBUS; break; }
      if (cmd_go_test)    { cmd_go_test = 0; control_reset(); robot_state = ST_TEST; break; }
      if (cmd_seq_start && seq_count > 0) {
        cmd_seq_start = 0; seq_state = SEQ_IDLE; seq_pair_idx = 0;
        robot_state = ST_SEQUENCE; break;
      }

      /* Jog: relative offset */
      if (cmd_jog_deg != 0.0f) {
        float tgt = robot_pos_deg + cmd_jog_deg;
        tgt = fmodf(tgt, 360.0f);
        if (tgt < 0.0f) tgt += 360.0f;
        cmd_jog_deg = 0.0f;
        do_move_deg(tgt);
      }

      /* P2P edge-triggered */
      if (cmd_p2p_deg != cmd_p2p_last) {
        /* already snapshotted in modbus_app_receive */
        do_move_deg(cmd_p2p_deg);
      }

      /* Set home */
      if (cmd_set_home) { cmd_set_home = 0; hardware_finish_homing(); }

      handle_actuator_cmd();
      break;
    }

    /* ── SEQUENCE: automatic pick/place ─────────────────────────────────── */
    case ST_SEQUENCE:
      if (cmd_soft_stop)  { cmd_soft_stop = 0; seq_state = SEQ_IDLE; seq_pair_idx = 0; control_reset(); hardware_set_motor(0.0f); robot_state = ST_AUTO; break; }
      if (cmd_do_home)    { cmd_do_home = 0; seq_state = SEQ_IDLE; seq_pair_idx = 0; control_reset(); hardware_set_motor(0.0f); hardware_homing_sensor_enable(); robot_state = ST_HOMING_FAST; break; }
      sequence_tick();
      break;

    /* ── TEST: placeholder — ctrl fills in motion logic ─────────────────── */
    case ST_TEST:
      if (cmd_soft_stop || cmd_do_home || cmd_go_auto) {
        cmd_soft_stop = cmd_go_auto = 0;
        if (cmd_do_home) { cmd_do_home = 0; control_reset(); hardware_homing_sensor_enable(); robot_state = ST_HOMING_FAST; }
        else { control_reset(); hardware_set_motor(0.0f); robot_state = ST_AUTO; }
      }
      /* ctrl layer drives the motor during TEST */
      break;

    default:
      hardware_set_motor(0.0f);
      robot_state = ST_IDLE;
      break;
  }

  /* Power button hold 3s → shutdown */
  {
    static uint32_t pwr_hold = 0;
    if (robot_power_btn) {
      if (++pwr_hold >= 300) hardware_set_power_latch(0); /* 3s × 100Hz */
    } else {
      pwr_hold = 0;
    }
  }

  update_tower_lights(tick);
}
