#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

/* =========================================================================
 * robot.h — Shared Robot State
 *
 * Single source of truth for all application-level state.
 * Written by: hardware.c (sensor reads), state_machine.c (state), ISR (motion)
 * Read by:    modbus_app.c, state_machine.c, hardware.c
 * ========================================================================= */

/* ── Robot states ─────────────────────────────────────────────────────────
 *
 *  Hierarchy:
 *    ST_EMER           — always wins (ESTOP or joystick LB)
 *    ST_MANUAL_SWITCH  — cabinet switch HIGH: joystick motor control only
 *    everything else   — cabinet switch LOW: full Modbus access
 */
typedef enum {
  ST_INIT           = 0,
  ST_IDLE           = 1,
  ST_HOMING_FAST    = 2,   // direct PWM left 25%
  ST_HOMING_BACKOFF = 3,   // direct PWM right 20% for 1s
  ST_HOMING_SLOW    = 4,   // direct PWM left 6%
  ST_MANUAL_SWITCH  = 5,   // cabinet switch HIGH — joystick controls motor
  ST_MANUAL_MODBUS  = 6,   // BaseSystem MANUAL tab — jog + pneumatics
  ST_AUTO           = 7,   // BaseSystem AUTO tab — P2P moves
  ST_SEQUENCE       = 8,   // running pick/place sequence
  ST_TEST           = 9,   // BaseSystem TEST tab
  ST_EMER           = 10   // emergency (ESTOP pin LOW or joystick LB)
} RobotState_t;

/* ── State & homing ───────────────────────────────────────────────────────── */
extern volatile RobotState_t robot_state;
extern volatile uint8_t      robot_homed;       // 1 after successful homing

/* ── Motion (ISR writes 1kHz, 100Hz reads) ───────────────────────────────── */
extern volatile float        robot_pos_rad;     // encoder position (rad)
extern volatile float        robot_vel_rad_s;   // filtered velocity (rad/s)
extern volatile float        robot_acc_rad_s2;  // filtered acceleration
extern volatile float        robot_pos_deg;     // wrapped 0–359.99° for display
extern volatile float        robot_cumul_deg;   // cumulative from home (safety)
#define ROBOT_MAX_ROTATION_DEG  720.0f          // hard limit: ±2 full rotations

/* ── Motor output (100Hz writes, ISR reads) ──────────────────────────────── */
extern volatile float        robot_motor_pct;   // -1.0..+1.0 → hw_set_motor_pwm

/* ── Move command (fsm writes, ISR reads) ────────────────────────────────── */
extern volatile float        robot_target_rad;  // move target
extern volatile uint8_t      robot_move_armed;  // 1 = new target, ISR arms ctrl

/* ── Sensors (hardware.c writes, others read) ────────────────────────────── */
extern volatile uint8_t      robot_estop;       // 1 = ESTOP pressed (LOW)
extern volatile uint8_t      robot_reed_up;
extern volatile uint8_t      robot_reed_down;
extern volatile uint8_t      robot_reed_grip;
extern volatile uint8_t      robot_home_sensor; // 1 = home sensor active
extern volatile uint8_t      robot_reset_btn;   // 1 = button held
extern volatile uint8_t      robot_power_btn;
extern volatile uint8_t      robot_mode_sw;     // 1 = switch HIGH (MANUAL)
extern volatile float        robot_current_A;

/* ── Reed dummy (simulate reed when no hardware sensors) ─────────────────── */
extern volatile uint8_t      reed_dummy_en;         // 1 = use dummy delays
extern volatile float        reed_dummy_delay_ms;   // delay in ms

/* ── Homing parameters (Live Expressions tunable) ────────────────────────── */
extern volatile float        homing_fast_pct;        // PWM%, default 25
extern volatile float        homing_backoff_pct;     // PWM%, default 20
extern volatile uint32_t     homing_backoff_ticks;   // 1ms ticks, default 100
extern volatile float        homing_slow_pct;        // PWM%, default 6
extern volatile float        home_offset_deg;        // move after homing, default 0.6

/* ── Encoder direction ────────────────────────────────────────────────────── */
extern volatile uint8_t      encoder_inverted;  // Live Expressions only, NOT Modbus

/* ── Sequence data (modbus_app writes, state_machine reads) ─────────────── */
#define ROBOT_SEQ_MAX  8
extern volatile uint8_t      seq_count;
extern volatile float        seq_pick_deg [ROBOT_SEQ_MAX];
extern volatile float        seq_place_deg[ROBOT_SEQ_MAX];

/* ── Commands: modbus_app sets → state_machine executes → clears ─────────── */
extern volatile uint8_t      cmd_do_home;       // start homing sequence
extern volatile uint8_t      cmd_set_home;      // virtual home at current pos
extern volatile uint8_t      cmd_go_manual;     // enter ST_MANUAL_MODBUS
extern volatile uint8_t      cmd_go_auto;       // enter ST_AUTO
extern volatile uint8_t      cmd_go_test;       // enter ST_TEST
extern volatile float        cmd_jog_deg;       // relative jog (0=none)
extern volatile float        cmd_p2p_deg;       // P2P target (edge-triggered)
extern volatile float        cmd_p2p_last;      // previous value for edge detect
extern volatile uint8_t      cmd_seq_start;     // start loaded sequence
extern volatile uint8_t      cmd_seq_pick;      // manual: do pick step
extern volatile uint8_t      cmd_seq_place;     // manual: do place step
extern volatile uint8_t      cmd_soft_stop;     // hold current position now
extern volatile uint8_t      cmd_pause;         // pause trajectory
extern volatile uint8_t      cmd_actuator;      // one-hot: b0=cyl b1=grip-open b2=grip-close

/* ── Control parameters (used by control.h implementor) ─────────────────── */
typedef struct {
  float kp_vel, ki_vel, kd_vel;   // velocity PID
  float kp_pos, ki_pos, kd_pos;   // position PID
  float v_max, a_max, j_max;      // trajectory (rad/s, rad/s², rad/s³)
  uint8_t traj_type;              // 0=trapezoid  1=s-curve  2=direct
} ControlModel_t;

#define CONTROL_MODEL_COUNT  2
extern ControlModel_t        control_model[CONTROL_MODEL_COUNT];
extern volatile float        control_model_switch_deg; // displacement threshold

/* ── System config ───────────────────────────────────────────────────────── */
extern volatile float        sys_max_speed;   // 0..1 caps motor output
extern volatile float        sys_V_supply;    // bus voltage (V)
extern volatile uint8_t      sys_ff_enabled;  // feedforward enable

/* ── ISR / timing flags ──────────────────────────────────────────────────── */
extern volatile uint8_t      flag_10ms;             // set by ISR every 10 ticks
extern volatile uint8_t      encoder_reset_req;     // request ISR to zero encoder
extern volatile uint8_t      g_homing_sensor_pending; // set by EXTI, cleared by ISR

#endif /* __ROBOT_H */
