# Developer Guide — 1-DOF Robot Control System

เอกสารสำหรับนักพัฒนาและทีม Control ที่จะ implement อัลกอริทึมควบคุม

---

## Live Expressions Dashboard

Flash โค้ดด้วยปุ่ม **Debug (🐞)** แล้ว Add expression: `dev_dash`

```
▼ dev_dash
  ▼ Ctrl                          ← ปรับค่า runtime ได้ทั้งหมด
      mode             = 0        ← 0=Production, 1=HW Test, 2=Joy Test, 3=Auto Test
      force_pneumatic  = 0
      force_gripper    = 0
      force_tower_green/yellow/red = 0
      force_emer_out   = 0
      force_motor_speed = 0.0     ← ใช้ใน mode 1 (-1.0 ถึง +1.0)
      ramp_rate        = 0.03     ← Slew rate /10ms (0.01=ช้า, 0.10=เร็ว)
      max_speed        = 0.40     ← Hard cap (0.0-1.0)
      auto_speed       = 0.30     ← Speed สำหรับ mode 3
      auto_period_fwd_ms = 1000   ← ms ทิศ Forward ใน mode 3
      auto_period_rev_ms = 1000   ← ms ทิศ Reverse ใน mode 3
      cur_zero_v       = 2.50     ← Zero-current voltage (วัดจาก multimeter)
      cur_sens         = 0.066    ← Sensitivity V/A (66mV/A สำหรับ WCS1800)
  ▼ Joy
      connected        = 0/1
      raw_buttons      = 0x0000
      L_Y              = 0.0      ← -1.0 ถึง +1.0
      R_T              = 0.0
      L_T              = 0.0
  ▼ Status
      state            = IDLE     ← State Machine state ปัจจุบัน
      motor_cmd        = 0.0      ← Speed สุทธิที่ส่งออกจริง (signed)
      encoder          = 0        ← Absolute position (counts, TI12 mode)
      current_A        = 0.0000   ← กระแสมอเตอร์ (Amperes, float)
      pos_rad          = 0.0      ← ตำแหน่ง (rad) — ตรงกับที่ส่งไป Simulink
      vel_rad_s        = 0.0      ← ความเร็ว (rad/s)
      acc_rad_s2       = 0.0      ← ความเร่ง (rad/s²)
  ▼ In
      estop            = 1        ← 1=ปกติ, 0=กด
      mode_switch      = 0/1
      reset            = 1
      power            = 1
  ▼ Out
      pwm/dir/pneumatic/gripper/tower_g/y/r/reset_led/emer
```

---

## โหมดทดสอบ Hardware (Mode 1)

ตั้ง `dev_dash.Ctrl.mode = 1` แล้วสั่ง output โดยตรง:

```
force_motor_speed = 0.3    → มอเตอร์หมุน Forward 30%
force_motor_speed = -0.3   → มอเตอร์หมุน Reverse 30%
force_pneumatic = 1        → กระบอกลม ON
force_gripper = 1          → Gripper ON
force_tower_red = 1        → ไฟแดง ON
```

**Note:** max_speed ยังคง cap อยู่ แม้ใน mode 1

---

## โหมดทดสอบ Auto Motor (Mode 3)

ตั้ง `dev_dash.Ctrl.mode = 3`

| Parameter | Default | คำอธิบาย |
|-----------|---------|---------|
| `auto_speed` | 0.30 | ความเร็ว (0.0-1.0) |
| `auto_period_fwd_ms` | 1000 | เวลาหมุน Forward (ms) |
| `auto_period_rev_ms` | 1000 | เวลาหมุน Reverse (ms) |
| `ramp_rate` | 0.03 | Slew rate (/10ms) |
| `max_speed` | 0.40 | Hard cap |

ตัวอย่าง: `fwd=2000, rev=500, auto_speed=0.25` → หน้า 2s, หลัง 0.5s ที่ 25%

---

## Motor Control Tuning

### Slew Rate (ramp_rate)
- ป้องกัน inrush current และ back-EMF
- ค่าต่ำ = เร่งช้า, ค่าสูง = เร่งเร็ว
- แนะนำ: เริ่มที่ 0.03, เพิ่มขึ้นทีละ 0.01 จนพบค่าที่ PSU ไม่ตัด

### Max Speed (max_speed)
- Cap สูงสุดที่มอเตอร์จะหมุน
- เพิ่มทีละ 0.05 พร้อมดู `current_A` ว่าไม่เกิน 15A

### Dead-time บน Direction Change
- ปัจจุบัน hardcode ไว้ที่ 5 cycles = 50ms
- ป้องกัน back-EMF spike เมื่อสลับทิศ

---

## Current Sensor Calibration

WCS1800 บน PC4 (ADC2_IN5), VCC=5V

**ขั้นตอน Calibrate:**
1. ตัดไฟมอเตอร์ออก (ไม่มีกระแสไหล)
2. วัด voltage ที่ขา OUT ของ WCS1800 ด้วย Multimeter
3. ตั้งค่า `dev_dash.Ctrl.cur_zero_v = <ค่าที่วัดได้>`
4. ตรวจสอบ `dev_dash.Status.current_A` ควรใกล้ 0 เมื่อไม่มีกระแส
5. ปล่อยกระแส Known เช่น 5A แล้วตรวจ `current_A` ถ้าเพี้ยนให้ปรับ `cur_sens`

**ค่าทฤษฎี WCS1800 @ 5V:**
- `cur_zero_v = 2.50`
- `cur_sens = 0.066` (66mV/A)

---

## Joystick Test Mode (Mode 2) — Button Layout ปัจจุบัน

| ปุ่ม | Output | Pin |
|------|--------|-----|
| BTN_A | EMER_OUTPUT | PB14 |
| BTN_B | POWER_LATCH | PB6 |
| BTN_X | PNEUMATIC | PC6 |
| BTN_Y | GRIPPER | PB11 |
| DPAD_UP | TOWER_G | PC7 |
| DPAD_DOWN | TOWER_Y | PB7 |
| DPAD_LEFT | TOWER_R | PC8 |
| DPAD_RIGHT | RESET_LED | PB4 |
| RT (กำ) + LY | มอเตอร์ | PA6 PWM |

---

## STATE_AUTO — Trajectory + Cascade PID (Implemented ✅)

ใช้ SCURVE.c/h + TRAPEZOID.c/h จาก Control Team integrate เข้า STATE_AUTO แล้ว

### วิธีใช้ (ผ่าน Live Expressions)

```
▼ dev_dash
  ▼ Auto
    — COMMANDS (write) —
    target_deg    = 90.0   ← เป้าหมาย (องศาจาก home)
    start_move    = 1      ← set 1 เพื่อ trigger (auto-clear)
    traj_type     = 0      ← 0=Trapezoid, 1=S-Curve, 2=Direct PID (ไม่มี trajectory)
    time_mode     = 0      ← 0=constraint-based, 1=time-based
    — Trajectory limits (time_mode=0) —
    v_max         = 1.57   ← rad/s (~0.25 rev/s)
    a_max         = 3.14   ← rad/s²
    j_max         = 10.0   ← rad/s³ (S-Curve เท่านั้น)
    — Trajectory times (time_mode=1) —
    t1_seg        = 0.314  ← S-Curve: jerk duration (s)
    t2_seg        = 0.186  ← S-Curve: const-accel (s)
    t_acc_seg     = 0.500  ← Trapezoid: accel (s)
    t_cruise_seg  = 1.186  ← cruise duration (s)
    — PID gains —
    kp_vel = 5.0   ki_vel = 0.1   kd_vel = 0.0
    kp_pos = 1.0   ki_pos = 0.0   kd_pos = 0.0
    — STATUS (read) —
    pos_rad       ← position ปัจจุบัน (rad)
    vel_rad_s     ← velocity ปัจจุบัน (rad/s)
    pos_ideal     ← trajectory reference (rad)
    pos_err       ← position error (rad)
    vel_sp        ← velocity setpoint (rad/s)
    pwm_out       ← motor command ออก (-1.0 ถึง 1.0)
    traj_active   ← 1=กำลังวิ่ง, 0=หยุดแล้ว
```

### ขั้นตอนทดสอบ (แนะนำเริ่มจาก traj_type=2)

**Phase 1 — Direct Position PID (traj_type=2):** เร็วที่สุด ไม่มีตัวแปรของ trajectory
1. Enter STATE_AUTO ผ่าน Modbus `0x01 = 0x04` หรือ MODE switch
2. `traj_type = 2`, `kp_pos = 1.0`, `ki_pos = 0`, `kp_vel = 5`
3. `target_deg = 90` → `start_move = 1`
4. ดู `pos_err` และ `pwm_out` — ถ้าสั่นให้ลด `kp_vel`, ถ้าช้าให้เพิ่ม `kp_pos`
5. ค่อยๆ เพิ่ม `ki_pos` เพื่อกำจัด steady-state error

**Phase 2 — Trapezoidal (traj_type=0):** เพิ่ม smooth acceleration
1. ใช้ gain ที่ได้จาก Phase 1
2. `traj_type = 0`, ตั้ง `v_max`, `a_max` ตามต้องการ
3. `target_deg = 90` → `start_move = 1`

**Phase 3 — S-Curve (traj_type=1):** smoothest
1. เพิ่ม `j_max` (default 10 rad/s³)

### ⚠️ Note: Loop Rate Difference
- Control Team tune gains ที่ **1kHz**
- เราใช้ **100Hz** (main loop)
- ผลกระทบ: `kp_vel=30` ของเพื่อนอาจ oscillate ในระบบของเรา
- แนะนำเริ่ม `kp_vel=5` แล้วค่อยเพิ่ม

### ⚠️ Note: เพิ่มไฟล์ใน CubeIDE
ต้อง Add `SCURVE.c` และ `TRAPEZOID.c` เข้า build:
Project Explorer → คลิกขวา `Core/Src` → `Add Existing Files`

### STATE_CALIBRATE (Homing — ยังต้อง implement)
```c
case STATE_CALIBRATE:
    motor_speed_cmd = -0.1f; // เคลื่อนไปหา Home sensor
    // เมื่อ HOME sensor trigger:
    //   current_position = 0;
    //   current_state = STATE_IDLE;
    break;
```

### Global Variables ที่ใช้ได้
```c
extern float    motor_speed_cmd;   // สั่งความเร็ว (-1.0 ถึง +1.0)
extern int32_t  current_position;  // encoder counts (อัพเดททุก 1ms)
extern float    current_sensor_A;  // กระแส Amperes (อัพเดททุก 10ms)
extern float    ctrl_vel_rad_s;    // velocity (rad/s, windowed @1kHz)
```

---

## Safety Features

| Feature | Implementation |
|---------|--------------|
| IWDG Watchdog | 3 วินาที timeout, refresh ใน 100Hz loop |
| E-Stop Hardware | PB13 EXTI Falling → STATE_EMER + PWM=0 ทันที |
| E-Stop Software | L3+R3 บนจอย → STATE_EMER |
| Slew Rate | 50ms dead-time บน direction change |
| Current Monitor | `current_A` available สำหรับ Control Team ใช้ implement overcurrent protection |
| Power Button | กดค้าง 3 วินาที → POWER_LATCH LOW → shutdown |
