"""
Refactor DevDashboard_t: flat struct → 6 named sub-structs
Run once from any directory: python tools/refactor_dash.py
"""
import re

MAIN_C = r"C:\PaYae\Microcon\ws\1-Dof-full-control\Core\Src\main.c"

# ── 1. Field → sub-struct mapping (longest first to avoid partial-match) ──
RENAMES = [
    # Cmd
    ("dev_dash.target_deg",        "dev_dash.Cmd.target_deg"),
    ("dev_dash.start_move",        "dev_dash.Cmd.start_move"),
    ("dev_dash.set_home",          "dev_dash.Cmd.set_home"),
    ("dev_dash.cancel_move",       "dev_dash.Cmd.cancel_move"),
    # Traj
    ("dev_dash.traj_type",         "dev_dash.Traj.traj_type"),
    ("dev_dash.time_mode",         "dev_dash.Traj.time_mode"),
    ("dev_dash.t_cruise_seg",      "dev_dash.Traj.t_cruise_seg"),
    ("dev_dash.t_acc_seg",         "dev_dash.Traj.t_acc_seg"),
    ("dev_dash.t1_seg",            "dev_dash.Traj.t1_seg"),
    ("dev_dash.t2_seg",            "dev_dash.Traj.t2_seg"),
    ("dev_dash.v_max",             "dev_dash.Traj.v_max"),
    ("dev_dash.a_max",             "dev_dash.Traj.a_max"),
    ("dev_dash.j_max",             "dev_dash.Traj.j_max"),
    # Status
    ("dev_dash.pos_deg",           "dev_dash.Status.pos_deg"),
    ("dev_dash.pos_rad",           "dev_dash.Status.pos_rad"),
    ("dev_dash.vel_rad_s",         "dev_dash.Status.vel_rad_s"),
    ("dev_dash.acc_rad_s2",        "dev_dash.Status.acc_rad_s2"),
    ("dev_dash.pos_ideal",         "dev_dash.Status.pos_ideal"),
    ("dev_dash.vel_ideal",         "dev_dash.Status.vel_ideal"),
    ("dev_dash.pos_err",           "dev_dash.Status.pos_err"),
    ("dev_dash.vel_sp",            "dev_dash.Status.vel_sp"),
    ("dev_dash.pwm_out",           "dev_dash.Status.pwm_out"),
    ("dev_dash.traj_active",       "dev_dash.Status.traj_active"),
    ("dev_dash.current_A",         "dev_dash.Status.current_A"),
    ("dev_dash.encoder_raw",       "dev_dash.Status.encoder_raw"),
    ("dev_dash.motor_cmd",         "dev_dash.Status.motor_cmd"),
    ("dev_dash.status_state",      "dev_dash.Status.status_state"),
    # Sys
    ("dev_dash.telemetry_mode",    "dev_dash.Sys.telemetry_mode"),
    ("dev_dash.max_speed",         "dev_dash.Sys.max_speed"),
    ("dev_dash.ramp_rate",         "dev_dash.Sys.ramp_rate"),
    ("dev_dash.acc_alpha",         "dev_dash.Sys.acc_alpha"),
    ("dev_dash.cur_zero_v",        "dev_dash.Sys.cur_zero_v"),
    ("dev_dash.cur_sens",          "dev_dash.Sys.cur_sens"),
    ("dev_dash.mode",              "dev_dash.Sys.mode"),
    # IO — inputs
    ("dev_dash.in_estop",          "dev_dash.IO.in_estop"),
    ("dev_dash.in_mode",           "dev_dash.IO.in_mode"),
    ("dev_dash.in_reset",          "dev_dash.IO.in_reset"),
    ("dev_dash.in_power",          "dev_dash.IO.in_power"),
    # IO — outputs
    ("dev_dash.out_pneumatic",     "dev_dash.IO.out_pneumatic"),
    ("dev_dash.out_gripper",       "dev_dash.IO.out_gripper"),
    ("dev_dash.out_tower_g",       "dev_dash.IO.out_tower_g"),
    ("dev_dash.out_tower_y",       "dev_dash.IO.out_tower_y"),
    ("dev_dash.out_tower_r",       "dev_dash.IO.out_tower_r"),
    ("dev_dash.out_reset_led",     "dev_dash.IO.out_reset_led"),
    ("dev_dash.out_emer",          "dev_dash.IO.out_emer"),
    ("dev_dash.out_pwm",           "dev_dash.IO.out_pwm"),
    ("dev_dash.out_dir",           "dev_dash.IO.out_dir"),
    # IO — joystick
    ("dev_dash.joy_connected",     "dev_dash.IO.joy_connected"),
    ("dev_dash.joy_buttons",       "dev_dash.IO.joy_buttons"),
    ("dev_dash.joy_ly",            "dev_dash.IO.joy_ly"),
    ("dev_dash.joy_lt",            "dev_dash.IO.joy_lt"),
    ("dev_dash.joy_rt",            "dev_dash.IO.joy_rt"),
    # Test
    ("dev_dash.force_pneumatic",   "dev_dash.Test.force_pneumatic"),
    ("dev_dash.force_gripper",     "dev_dash.Test.force_gripper"),
    ("dev_dash.force_tower_g",     "dev_dash.Test.force_tower_g"),
    ("dev_dash.force_tower_y",     "dev_dash.Test.force_tower_y"),
    ("dev_dash.force_tower_r",     "dev_dash.Test.force_tower_r"),
    ("dev_dash.force_motor",       "dev_dash.Test.force_motor"),
    ("dev_dash.force_emer",        "dev_dash.Test.force_emer"),
    ("dev_dash.test_period_fwd_ms","dev_dash.Test.test_period_fwd_ms"),
    ("dev_dash.test_period_rev_ms","dev_dash.Test.test_period_rev_ms"),
    ("dev_dash.test_speed",        "dev_dash.Test.test_speed"),
]

# ── 2. New struct + init block (replaces old typedef struct … DevDashboard_t + init) ──
NEW_TYPEDEF = r"""/* =========================================================================
 * DevDashboard — 6 sub-structs; each appears as a named folder
 *   in Live Expressions: dev_dash → Cmd / Status / Traj / Sys / IO / Test
 * ========================================================================= */

/* ── Cmd: คำสั่ง P2P (เขียน) ────────────────────────────────────────────── */
typedef struct {
  float    target_deg;    // เป้าหมาย (degrees จาก home)
  uint8_t  start_move;    // Set 1 → สั่ง move (auto-clear)
  uint8_t  set_home;      // Set 1 → zero encoder + homed=1 (auto-clear)
  uint8_t  cancel_move;   // Set 1 → stop ทันที (auto-clear)
} DashCmd_t;

/* ── Status: สถานะ real-time (อ่าน) ─────────────────────────────────────── */
typedef struct {
  float        pos_deg;         // ตำแหน่ง (degrees)
  float        pos_rad;         // ตำแหน่ง (rad)
  float        vel_rad_s;       // ความเร็ว (rad/s)
  float        acc_rad_s2;      // ความเร่ง (rad/s²)
  float        pos_ideal;       // trajectory ref pos (rad)
  float        vel_ideal;       // trajectory ref vel (rad/s)
  float        pos_err;         // position error (rad)
  float        vel_sp;          // vel setpoint จาก pos loop (rad/s)
  float        pwm_out;         // motor command -1..+1
  uint8_t      traj_active;     // 1 = trajectory กำลังทำงาน
  float        current_A;       // กระแส (A)
  int32_t      encoder_raw;     // encoder counts (raw)
  float        motor_cmd;       // motor speed command (mirror)
  RobotState_t status_state;    // current state
} DashStatus_t;

/* ── Traj: Trajectory parameters ─────────────────────────────────────────── */
typedef struct {
  uint8_t  traj_type;     // 0=Trapezoid  1=S-Curve  2=Direct
  uint8_t  time_mode;     // 0=constraint-based  1=time-based
  float    v_max;         // ความเร็วสูงสุด (rad/s)
  float    a_max;         // ความเร่งสูงสุด (rad/s²)
  float    j_max;         // jerk สูงสุด (rad/s³, S-Curve)
  float    t_acc_seg;     // ช่วงเร่ง (s)
  float    t_cruise_seg;  // ช่วง cruise (s)
  float    t1_seg;        // S-Curve jerk segment (s)
  float    t2_seg;        // S-Curve const-accel segment (s)
} DashTraj_t;

/* ── Sys: System config ──────────────────────────────────────────────────── */
typedef struct {
  SystemMode_t mode;          // 0=Production 1=HW_Test 2=Joy_Test 3=Motor_Test
  float    max_speed;         // speed cap 0.0–1.0
  float    ramp_rate;         // slew rate per 10ms (0.01=ช้า, 0.1=เร็ว)
  float    acc_alpha;         // velocity filter alpha (0.1=smooth, 1.0=raw)
  float    cur_zero_v;        // current sensor zero voltage
  float    cur_sens;          // sensitivity V/A (0.066)
  uint8_t  telemetry_mode;    // 0=Modbus  1=Simulink via LPUART1
} DashSys_t;

/* ── IO: Raw GPIO + Joystick (อ่าน) ─────────────────────────────────────── */
typedef struct {
  uint8_t  in_estop;          uint8_t  in_mode;
  uint8_t  in_reset;          uint8_t  in_power;
  uint8_t  out_pwm;           uint8_t  out_dir;
  uint8_t  out_pneumatic;     uint8_t  out_gripper;
  uint8_t  out_tower_g;       uint8_t  out_tower_y;  uint8_t  out_tower_r;
  uint8_t  out_reset_led;     uint8_t  out_emer;
  uint8_t  joy_connected;     float    joy_ly;
  float    joy_lt;            float    joy_rt;
  uint16_t joy_buttons;
} DashIO_t;

/* ── Test: HW test overrides (mode=1 or 3) ──────────────────────────────── */
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
  DashCmd_t    Cmd;     // คำสั่ง P2P (เขียน)
  DashStatus_t Status;  // สถานะ real-time (อ่าน)
  DashTraj_t   Traj;    // Trajectory params
  DashSys_t    Sys;     // System config
  DashIO_t     IO;      // Raw GPIO + Joystick
  DashTest_t   Test;    // Test mode overrides
} DevDashboard_t;

DevDashboard_t dev_dash = {
  .Sys.mode            = SYS_MODE_PRODUCTION,
  .Sys.ramp_rate       = 0.05f,
  .Sys.max_speed       = 0.40f,
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
};"""

# ── regex that matches the old typedef+init block ──
OLD_BLOCK_RE = re.compile(
    r"/\* =+\s*\* DevDashboard.*?^DevDashboard_t dev_dash\s*=\s*\{.*?^};",
    re.DOTALL | re.MULTILINE,
)

with open(MAIN_C, "r", encoding="utf-8") as f:
    src = f.read()

# Step 1 — replace struct + init block
new_src, n_block = OLD_BLOCK_RE.subn(NEW_TYPEDEF, src)
if n_block != 1:
    print(f"ERROR: expected 1 block match, got {n_block}")
    exit(1)
print(f"  struct block replaced ({n_block})")

# Step 2 — rename field references
total = 0
for old, new in RENAMES:
    count = new_src.count(old)
    new_src = new_src.replace(old, new)
    if count:
        print(f"  {old:45s} -> {new}  ({count}x)")
    total += count
print(f"\nTotal references renamed: {total}")

# Step 3 — verify no stale flat refs remain
remaining = [r[0] for r in RENAMES if r[0] in new_src]
if remaining:
    print("\nWARNING — stale flat refs still found:")
    for r in remaining:
        print(f"  {r}")
else:
    print("Verification OK — no stale flat refs found")

with open(MAIN_C, "w", encoding="utf-8") as f:
    f.write(new_src)
print("main.c written successfully")
