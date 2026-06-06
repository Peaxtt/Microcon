#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <stdint.h>
#include "ENCODER.h"
#include "SCURVE.h"
#include "TRAPEZOID.h"
#include "REF_FEEDFORWARD.h"
#include "DistFF.h"
#include "control.h"

/* ── HAL handles (owned by main.c / CubeMX) ─────────────────────────────── */
extern TIM_HandleTypeDef   htim1;   /* encoder TI12 */
extern TIM_HandleTypeDef   htim3;   /* PWM CH4 */
extern TIM_HandleTypeDef   htim7;   /* 1kHz ISR */
extern ADC_HandleTypeDef   hadc2;   /* current sensor DMA */
extern IWDG_HandleTypeDef  hiwdg;
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef  hlpuart1;
extern UART_HandleTypeDef  huart3;
extern DMA_HandleTypeDef   hdma_adc2;
extern DMA_HandleTypeDef   hdma_lpuart1_rx;
extern DMA_HandleTypeDef   hdma_lpuart1_tx;
extern DMA_HandleTypeDef   hdma_usart3_rx;

/* ── Enumerations ────────────────────────────────────────────────────────── */
typedef enum {
  STATE_INIT        = 0,
  STATE_IDLE        = 1,
  STATE_HOMING_FAST    = 2,
  STATE_HOMING_BACKOFF = 3,
  STATE_HOMING_SLOW    = 4,
  STATE_MANUAL      = 5,
  STATE_MANUAL_MB   = 6,
  STATE_AUTO        = 7,
  STATE_SEQUENCE    = 8,
  STATE_TEST        = 9,
  STATE_EMER        = 10
} RobotState_t;

typedef enum {
  SEQ_MB_IDLE = 0,
  SEQ_MB_GOING_PICK,
  SEQ_MB_PICKING,
  SEQ_MB_GOING_PLACE,
  SEQ_MB_PLACING,
} SeqMBState_t;

typedef enum { TEST_IDLE=0, TEST_GOING_END, TEST_GOING_START } TestState_t;

typedef enum { JSEQ_IDLE=0, JSEQ_PK1, JSEQ_PK2, JSEQ_PK3, JSEQ_PL1, JSEQ_PL2 } JoySeqState_t;

typedef enum {
  ACT_IDLE = 0,
  ACT_PK_DOWN, ACT_PK_GRIP, ACT_PK_UP,
  ACT_PL_DOWN, ACT_PL_OPEN, ACT_PL_UP,
} ActSeqState_t;

typedef enum {
  SYS_MODE_PRODUCTION = 0,
  SYS_MODE_HARDWARE_TEST = 1,
  SYS_MODE_JOYSTICK_TEST = 2,
  SYS_MODE_AUTO_MOTOR_TEST = 3
} SystemMode_t;

/* ── DevDashboard structs ────────────────────────────────────────────────── */
typedef struct {
  float    target_deg;
  uint8_t  start_move;
  uint8_t  set_home;
  uint8_t  cancel_move;
} DashCmd_t;

typedef struct {
  float        pos_deg;
  float        pos_rad;
  float        vel_rad_s;
  float        acc_rad_s2;
  float        pos_ideal;
  float        vel_ideal;
  float        pos_err;
  float        vel_sp;
  float        pwm_out;
  uint8_t      traj_active;
  float        current_A;
  int32_t      encoder_raw;
  float        motor_cmd;
  RobotState_t status_state;
} DashStatus_t;

typedef struct {
  uint8_t  traj_type;
  uint8_t  time_mode;
  float    v_max;
  float    a_max;
  float    j_max;
  float    t_acc_seg;
  float    t_cruise_seg;
  float    t1_seg;
  float    t2_seg;
} DashTraj_t;

typedef struct {
  SystemMode_t mode;
  float    max_speed;
  float    ramp_rate;
  float    acc_alpha;
  float    cur_zero_v;
  float    cur_sens;
  uint8_t  telemetry_mode;
} DashSys_t;

typedef struct {
  uint8_t  in_estop;          uint8_t  in_mode;
  uint8_t  in_reset;          uint8_t  in_power;
  uint8_t  out_pwm;           uint8_t  out_dir;
  uint8_t  out_power_latch;   uint8_t  out_pneumatic;  uint8_t  out_gripper;
  uint8_t  out_tower_g;       uint8_t  out_tower_y;    uint8_t  out_tower_r;
  uint8_t  out_reset_led;     uint8_t  out_emer;
  uint8_t  reed_up;           uint8_t  reed_down;      uint8_t  reed_grip;
  uint8_t  joy_connected;     float    joy_ly;
  float    joy_lt;            float    joy_rt;
  uint16_t joy_buttons;
} DashIO_t;

typedef struct {
  float    force_motor;
  uint8_t  force_pneumatic;   uint8_t  force_gripper;
  uint8_t  force_tower_g;     uint8_t  force_tower_y;
  uint8_t  force_tower_r;     uint8_t  force_emer;
  float    test_speed;
  uint16_t test_period_fwd_ms;
  uint16_t test_period_rev_ms;
} DashTest_t;

typedef struct {
  uint8_t  control_mode;
  uint8_t  traj_type;
  uint8_t  pid_enabled;
  uint8_t  start_move;
  uint8_t  reset_all;
  uint8_t  zero_encoder;
  uint8_t  apply_motor_params;

  float    traj_target_deg;
  float    max_velocity;
  float    max_accel;
  float    max_jerk;
  float    t1_seg;
  float    t2_seg;
  float    t_acc_seg;
  float    t_cruise_seg;
  float    t_dec_seg;

  float    kp_position;
  float    ki_pos;
  float    kd_pos;
  float    pos_deadband_deg;
  float    vel_deadband;
  float    pos_integral_live;

  float    kp_vel;
  float    ki_vel;
  float    kd_vel;
  float    min_pwm_threshold;

  float    motor_J_eq;
  float    motor_b_eq;
  float    motor_N;
  float    motor_Kt;
  float    motor_Ke;
  float    motor_L;
  float    motor_R_arm;

  uint8_t  refff_enabled;
  float    V_supply;
  uint8_t  distff_enabled;
  float    distff_tau;
  float    g_distff_pwm;

  uint8_t  kalman_enabled;
  float    kf_meas_noise;
  float    kf_proc_vel_noise;
  float    g_kf_velocity;
  float    g_kf_position;

  float    kf4_sigma_v2;
  float    kf4_r_theta;
  float    g_kf4_position;
  float    g_kf4_velocity;
  float    g_kf4_current;
  float    g_kf4_tau_d_obs;

  float    kf_cal_pwm;
  uint8_t  kf_sine_enabled;
  float    kf_sine_amp;
  float    kf_sine_freq;
  float    kf_sine_dir;

  float    current_zero_counts;
  float    current_counts_per_amp;
  uint32_t adc_raw_current;
  float    g_current_A;

  float    ss_error_deg;
  float    ss_error_rad;
  uint8_t  ss_reached;
  float    ss_final_error_deg;
  float    vel_error_live;
  float    i_term_live;
  float    g_traj_pos;
  float    g_smooth_vel;
  float    g_smooth_accel;
  float    g_jerk;
  float    g_position_rad;
  float    g_velocity_rad_s;
  float    g_pwm_duty;
  float    g_motor_voltage;

  uint8_t  motor_dir_inverted;
  uint8_t  gripper_updown;
  uint8_t  gripper_openclose;
  uint8_t  emer_output_ctrl;

  float    seq_targets[9];
  uint8_t  seq_count_ctrl;
  uint8_t  seq_enabled;
  uint8_t  seq_index;
  float    seq_step_delay;

  uint8_t  sweep_enabled;
  float    sweep_start_deg;
  float    sweep_stop_deg;
  float    sweep_step_deg;
  float    sweep_delay_s;
  uint8_t  sweep_index;
} DashCtrl_t;

typedef struct {
  DashCmd_t    Cmd;
  DashStatus_t Status;
  DashTraj_t   Traj;
  DashSys_t    Sys;
  DashIO_t     IO;
  DashTest_t   Test;
  DashCtrl_t   Ctrl;
} DevDashboard_t;

/* ── Global variables (defined in robot.c) ───────────────────────────────── */
extern RobotState_t current_state;

extern volatile float    motor_speed_cmd;
extern volatile int32_t  current_position;
extern volatile float    current_sensor_A;

extern uint32_t power_btn_hold_ms;

extern SCurve_t    g_scurve;
extern Trapezoid_t g_trapezoid;
extern volatile float ctrl_vel_rad_s;
extern float ctrl_traj_start;
extern float ctrl_direct_target;
extern float ctrl_vel_integral;
extern float ctrl_pos_integral;
extern volatile uint8_t enc_reset_pending;
extern volatile float ctrl_acc_rad_s2;
extern uint8_t sethome_led_cnt;

extern uint8_t sub_loop_counter;
extern volatile uint8_t flag_10ms;

extern Encoder_t my_encoder;
extern volatile float vel_filtered;

/* robot_pos_rad, control_model, control_model_switch_deg,
   sys_ff_enabled, sys_V_supply — all declared in control.h above */
extern volatile float    min_pwm_pct;
extern volatile float    ff_gain;

extern volatile float seq_delay_s;
extern volatile float seq_dwell_ms;
extern volatile float seq_timeout_ms;
extern volatile float seq_settle_ms;

extern volatile int8_t  soft_limit_dir;
extern volatile uint8_t enable_telemetry;

extern volatile uint8_t reed_up;
extern volatile uint8_t reed_down;
extern volatile uint8_t reed_grip;
extern volatile float   home_proximity_offset_deg;

extern volatile uint8_t reed_dummy_en;
extern volatile float   reed_dummy_delay_ms;
extern volatile float   reed_switch_delay_ms;

extern volatile uint8_t home_sensor_raw;
extern volatile float   cumulative_angle_deg;
extern volatile uint8_t homed;
extern volatile uint8_t homing_sensor_flag;
extern volatile uint8_t homing_sensor_pending;

extern RefFF_t  my_refff;
extern DistFF_t my_distff;
extern volatile uint8_t refff_enabled;
extern volatile uint8_t distff_enabled;
extern volatile float   V_supply;
extern volatile float   g_distff_tau_d;

extern volatile uint8_t  motor_dir_inverted;
extern volatile uint8_t  encoder_inverted;
extern volatile float    homing_fast_speed;
extern volatile float    homing_backoff_speed;
extern volatile uint32_t homing_backoff_ticks;
extern volatile float    home_offset_deg;
extern volatile float    homing_slow_speed;
extern volatile uint8_t  homing_final_zero_pending;
extern volatile uint8_t  homing_rezero_on_arrive;

extern DevDashboard_t dev_dash;

#define LOOP_DT     0.001f
#define RAD_PER_CNT COUNTS_TO_RAD   /* alias used by state_machine.c, main.c */

extern uint8_t skip_p2p_entry;

#endif /* ROBOT_H */
