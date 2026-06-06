#include "robot.h"

/* ── Robot state ─────────────────────────────────────────────────────────── */
RobotState_t current_state = STATE_INIT;

/* ── Motor & sensor globals ──────────────────────────────────────────────── */
volatile float   motor_speed_cmd  = 0.0f;
volatile int32_t current_position = 0;
volatile float   current_sensor_A = 0.0f;

uint32_t power_btn_hold_ms = 0;

/* ── Trajectory globals ──────────────────────────────────────────────────── */
SCurve_t    g_scurve;
Trapezoid_t g_trapezoid;
volatile float ctrl_vel_rad_s  = 0.0f;
float ctrl_traj_start          = 0.0f;
float ctrl_direct_target       = 0.0f;
float ctrl_vel_integral        = 0.0f;
float ctrl_pos_integral        = 0.0f;
volatile uint8_t enc_reset_pending = 0;
volatile float ctrl_acc_rad_s2 = 0.0f;
uint8_t sethome_led_cnt        = 0;

/* ── Loop timing ─────────────────────────────────────────────────────────── */
uint8_t sub_loop_counter   = 0;
volatile uint8_t flag_10ms = 0;

/* ── Encoder ─────────────────────────────────────────────────────────────── */
Encoder_t my_encoder;
volatile float vel_filtered = 0.0f;

/* ── Control interface ───────────────────────────────────────────────────── */
volatile float robot_pos_rad = 0.0f;
ControlModel_t control_model[CONTROL_MODEL_COUNT] = {
  { .kp_vel=10.0f, .ki_vel=0.01f, .kd_vel=0.0f,
    .kp_pos=2.0f,  .ki_pos=0.5f,  .kd_pos=0.15f,
    .v_max=4.0f, .a_max=12.56f, .j_max=10.0f, .traj_type=0 },
  { .kp_vel=10.0f, .ki_vel=0.01f, .kd_vel=0.0f,
    .kp_pos=2.0f,  .ki_pos=0.5f,  .kd_pos=0.15f,
    .v_max=4.0f, .a_max=12.56f, .j_max=10.0f, .traj_type=0 },
};
volatile float   control_model_switch_deg = 9999.0f;
volatile uint8_t sys_ff_enabled           = 1;   /* control.h externs this */
volatile float   sys_V_supply             = 24.0f; /* control.h externs this */
volatile float   min_pwm_pct              = 5.0f;
volatile float   ff_gain                  = 0.0f;

/* ── Sequence timing ─────────────────────────────────────────────────────── */
volatile float seq_delay_s    = 0.5f;
volatile float seq_dwell_ms   = 200.0f;
volatile float seq_timeout_ms = 3000.0f;
volatile float seq_settle_ms  = 300.0f;

volatile int8_t  soft_limit_dir   = 0;
volatile uint8_t enable_telemetry = 0;

/* ── Reed switches ───────────────────────────────────────────────────────── */
volatile uint8_t reed_up   = 0;
volatile uint8_t reed_down = 0;
volatile uint8_t reed_grip = 0;
volatile float   home_proximity_offset_deg = 0.0f;

volatile uint8_t reed_dummy_en        = 1;
volatile float   reed_dummy_delay_ms  = 30.0f;
volatile float   reed_switch_delay_ms = 500.0f;

/* ── Homing ──────────────────────────────────────────────────────────────── */
volatile uint8_t home_sensor_raw       = 0;
volatile float   cumulative_angle_deg  = 0.0f;
volatile uint8_t homed                 = 0;
volatile uint8_t homing_sensor_flag    = 0;
volatile uint8_t homing_sensor_pending = 0;

/* ── Feedforward ─────────────────────────────────────────────────────────── */
RefFF_t  my_refff;
DistFF_t my_distff;
volatile uint8_t refff_enabled  = 1;
volatile uint8_t distff_enabled = 0;
volatile float   V_supply       = 24.0f;
volatile float   g_distff_tau_d = 0.0f;

/* ── Motor/encoder direction & homing speeds ─────────────────────────────── */
volatile uint8_t  motor_dir_inverted   = 1;
volatile uint8_t  encoder_inverted     = 0;
volatile float    homing_fast_speed    = 0.25f;
volatile float    homing_backoff_speed = 0.10f;
volatile uint32_t homing_backoff_ticks = 100;
volatile float    home_offset_deg      = 0.0f;
volatile float    homing_slow_speed    = 0.09f;
volatile uint8_t  homing_final_zero_pending = 0;
volatile uint8_t  homing_rezero_on_arrive   = 0;

/* ── Live Expressions dashboard ──────────────────────────────────────────── */
DevDashboard_t dev_dash = {
  .Sys.mode            = SYS_MODE_PRODUCTION,
  .Sys.ramp_rate       = 0.05f,
  .Sys.max_speed       = 1.0f,
  .Sys.cur_zero_v      = 2.46f,
  .Sys.cur_sens        = 0.066f,
  .Sys.telemetry_mode  = 0,
  .Sys.acc_alpha       = 0.2f,
  .Traj.traj_type      = 0,
  .Traj.time_mode      = 0,
  .Traj.v_max          = 6.28f,
  .Traj.a_max          = 12.56f,
  .Traj.j_max          = 10.0f,
  .Traj.t1_seg         = 0.1f,
  .Traj.t2_seg         = 0.1f,
  .Traj.t_acc_seg      = 0.3f,
  .Traj.t_cruise_seg   = 0.2f,
  .Test.test_speed         = 0.30f,
  .Test.test_period_fwd_ms = 1000,
  .Test.test_period_rev_ms = 1000,
  .Ctrl.control_mode    = 3,
  .Ctrl.traj_type       = 0,
  .Ctrl.pid_enabled     = 1,
  .Ctrl.max_velocity    = 4.0f,
  .Ctrl.max_accel       = 12.56f,
  .Ctrl.max_jerk        = 10.0f,
  .Ctrl.kp_position     = 2.0f,
  .Ctrl.ki_pos          = 0.5f,
  .Ctrl.kd_pos          = 0.15f,
  .Ctrl.pos_deadband_deg = 0.35f,
  .Ctrl.vel_deadband     = 1.0f,
  .Ctrl.kp_vel          = 10.0f,
  .Ctrl.ki_vel          = 0.01f,
  .Ctrl.kd_vel          = 0.0f,
  .Ctrl.min_pwm_threshold = 0.1f,
  .Ctrl.motor_J_eq      = 0.027f,
  .Ctrl.motor_b_eq      = 0.5f,
  .Ctrl.motor_N         = 50.0f,
  .Ctrl.motor_Kt        = 0.00747f,
  .Ctrl.motor_Ke        = 0.0083f,
  .Ctrl.motor_L         = 0.0012794f,
  .Ctrl.motor_R_arm     = 2.8f,
  .Ctrl.refff_enabled      = 1,
  .Ctrl.V_supply           = 24.0f,
  .Ctrl.distff_enabled     = 0,
  .Ctrl.distff_tau         = 0.02f,
  .Ctrl.motor_dir_inverted = 1,
};
