#include "robot.h"

volatile RobotState_t robot_state        = ST_INIT;
volatile uint8_t      robot_homed        = 0;

volatile float        robot_pos_rad      = 0.0f;
volatile float        robot_vel_rad_s    = 0.0f;
volatile float        robot_acc_rad_s2   = 0.0f;
volatile float        robot_pos_deg      = 0.0f;
volatile float        robot_cumul_deg    = 0.0f;

volatile float        robot_motor_pct    = 0.0f;
volatile float        robot_target_rad   = 0.0f;
volatile uint8_t      robot_move_armed   = 0;

volatile uint8_t      robot_estop        = 0;
volatile uint8_t      robot_reed_up      = 0;
volatile uint8_t      robot_reed_down    = 0;
volatile uint8_t      robot_reed_grip    = 0;
volatile uint8_t      robot_home_sensor  = 0;
volatile uint8_t      robot_reset_btn    = 0;
volatile uint8_t      robot_power_btn    = 0;
volatile uint8_t      robot_mode_sw      = 0;
volatile float        robot_current_A    = 0.0f;

volatile uint8_t      reed_dummy_en        = 1;
volatile float        reed_dummy_delay_ms  = 30.0f;

volatile float        homing_fast_pct      = 0.25f;
volatile float        homing_backoff_pct   = 0.20f;
volatile uint32_t     homing_backoff_ticks = 100;
volatile float        homing_slow_pct      = 0.06f;
volatile float        home_offset_deg      = 0.6f;

volatile uint8_t      encoder_inverted  = 0;

volatile uint8_t      seq_count         = 0;
volatile float        seq_pick_deg [ROBOT_SEQ_MAX] = {0};
volatile float        seq_place_deg[ROBOT_SEQ_MAX] = {0};

volatile uint8_t      cmd_do_home       = 0;
volatile uint8_t      cmd_set_home      = 0;
volatile uint8_t      cmd_go_manual     = 0;
volatile uint8_t      cmd_go_auto       = 0;
volatile uint8_t      cmd_go_test       = 0;
volatile float        cmd_jog_deg       = 0.0f;
volatile float        cmd_p2p_deg       = 0.0f;
volatile float        cmd_p2p_last      = -9999.0f;
volatile uint8_t      cmd_seq_start     = 0;
volatile uint8_t      cmd_seq_pick      = 0;
volatile uint8_t      cmd_seq_place     = 0;
volatile uint8_t      cmd_soft_stop     = 0;
volatile uint8_t      cmd_pause         = 0;
volatile uint8_t      cmd_actuator      = 0;

ControlModel_t control_model[CONTROL_MODEL_COUNT] = {
  /* model[0] — fine/near: displacement < control_model_switch_deg */
  { .kp_vel=10.0f, .ki_vel=0.1f, .kd_vel=0.0f,
    .kp_pos= 1.0f, .ki_pos=5.0f, .kd_pos=0.1f,
    .v_max=5.0f, .a_max=12.56f, .j_max=10.0f,
    .traj_type=0 },
  /* model[1] — coarse/far: displacement >= control_model_switch_deg */
  { .kp_vel=10.0f, .ki_vel=0.1f, .kd_vel=0.0f,
    .kp_pos= 1.0f, .ki_pos=5.0f, .kd_pos=0.1f,
    .v_max=5.0f, .a_max=12.56f, .j_max=10.0f,
    .traj_type=0 },
};
volatile float   control_model_switch_deg = 30.0f;

volatile float   sys_max_speed   = 0.40f;
volatile float   sys_V_supply    = 24.0f;
volatile uint8_t sys_ff_enabled  = 1;

volatile uint8_t flag_10ms               = 0;
volatile uint8_t encoder_reset_req       = 0;
volatile uint8_t g_homing_sensor_pending = 0;
