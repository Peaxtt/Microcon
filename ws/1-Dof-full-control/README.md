# 1-DOF Robot Firmware — Control Team Handoff

> STM32G474RETx @ 170 MHz · NUCLEO-G474RE  
> Last updated: 2026-05-22

---

## สิ่งที่ต้องรู้ก่อน

- Firmware ทำงานบน **NUCLEO-G474RE** dev board
- Control loop **1 kHz** (TIM7 ISR) — Cascade PID + Trajectory generator
- Main loop **100 Hz** (flag_10ms) — State machine, Modbus, sensors
- PC เชื่อมผ่าน **Modbus RTU** 19200 8E1 Slave ID=21 (BaseSystem หรือ Python script)
- Joystick Xbox → RP2040-Zero → USART3 460800 bps

---

## Quick Start

```
1. STM32CubeIDE  →  Build All (Ctrl+B)  →  Run (F11)
2. LD2 กะพริบ 1 Hz = firmware ทำงาน
3. ต่อ BaseSystem หรือ joystick
4. กด Home ก่อนเสมอ (LT + X บน joystick หรือ Modbus reg 0x01 = 1)
5. หลัง home → กด P2P หรือสั่งผ่าน Modbus
```

---

## Live Expressions — ฉบับควบคุม

> เปิด **STM32CubeIDE → Debug → Live Expressions**  
> เพิ่มตัวแปรโดยพิมพ์ชื่อแล้วกด Enter

### ⭐ ตัวแปรที่ต้องใช้ทุกวัน

| ตัวแปร | ค่า default | ทำอะไร |
|--------|------------|--------|
| `current_state` | — | สถานะเครื่องตอนนี้ (อ่าน) |
| `homed` | 0 | 1 = home แล้ว. Set=1 เพื่อทดสอบ P2P โดยไม่ต้องมี sensor |
| `cumulative_angle_deg` | 0 | มุมจาก home (องศา, อ่าน) |
| `dev_dash.Auto.target_deg` | 0 | เป้าหมาย P2P (องศา, เขียน) |
| `dev_dash.Auto.start_move` | 0 | Set=1 → สั่ง move ทันที (auto-clear) |
| `dev_dash.Auto.set_home` | 0 | Set=1 → zero encoder + homed=1 |
| `ss_error_deg` | — | Position error ตอนนี้ (deg, อ่าน) |
| `pid_enabled` | 1 | 0 = หยุด PID + PWM=0 ทันที (safety kill switch) |

### 🎛️ PID Gains — แก้ตรงนี้เลย

> อยู่ใน `Core/Src/pid_control.c` · แก้ผ่าน Live Expressions เห็นผลทันที ไม่ต้อง re-flash

| ตัวแปร | ค่า default | คำอธิบาย |
|--------|------------|---------|
| `kp_vel` | 30.0 | Velocity loop — proportional |
| `ki_vel` | 0.1 | Velocity loop — integral |
| `kd_vel` | 0.0 | Velocity loop — derivative |
| `kp_pos` | 1.0 | Position loop — proportional |
| `ki_pos` | 0.0 | Position loop — integral |
| `kd_pos` | 0.0 | Position loop — derivative |
| `max_pwm` | 100.0 | PWM output clamp (%) |
| `vel_integral` | — | Integrator state (reset ได้ถ้าค้าง) |
| `vel_error_live` | — | Velocity error ตอนนี้ (rad/s, อ่าน) |
| `i_term_live` | — | Integral accumulator (อ่าน) |

> **วิธี reset integral:** Set `vel_integral = 0` และ `pos_integral = 0` ใน Live Expressions

### 📐 Trajectory Parameters

| ตัวแปร | ค่า default | คำอธิบาย |
|--------|------------|---------|
| `dev_dash.Auto.v_max` | 6.28 | ความเร็วสูงสุด (rad/s) |
| `dev_dash.Auto.a_max` | 12.56 | ความเร่งสูงสุด (rad/s²) |
| `dev_dash.Auto.j_max` | 10.0 | jerk สูงสุด (rad/s³, S-Curve เท่านั้น) |
| `dev_dash.Auto.t_acc_seg` | 0.3 | ช่วงเร่ง (s, time_mode=1) |
| `dev_dash.Auto.t_cruise_seg` | 0.2 | ช่วง cruise (s, time_mode=1) |
| `dev_dash.Auto.traj_type` | 0 | 0=Trapezoid, 1=S-Curve, 2=Direct |
| `dev_dash.Auto.time_mode` | 0 | 0=constraint-based, 1=time-based |

### 🔧 Motor & Direction

| ตัวแปร | ค่า default | คำอธิบาย |
|--------|------------|---------|
| `encoder_inverted` | 1 | 1 = กลับทิศ encoder (ไม่ต้องแก้สาย) |
| `motor_dir_inverted` | 0 | 1 = กลับทิศ PWM output |
| `fine_tune_speed` | 0.02 | ความเร็ว DPAD fine-tune (2%) |

### 🏠 Homing Parameters

| ตัวแปร | ค่า default | คำอธิบาย |
|--------|------------|---------|
| `homing_backoff_speed` | 0.20 | PWM ขณะ backoff ขวา (20%) |
| `homing_backoff_ticks` | 100 | จำนวน 10ms ticks สำหรับ backoff (100=1s) |
| `home_sensor_raw` | — | อ่านค่า HOME_SENSOR ตลอดเวลา (0/1, อ่าน) |

### 📡 Feedforward (optional)

| ตัวแปร | ค่า default | คำอธิบาย |
|--------|------------|---------|
| `refff_enabled` | 0 | 1 = เปิด model-based feedforward |
| `V_supply` | 24.0 | Bus voltage (V) สำหรับ feedforward |
| `distff_enabled` | 0 | 1 = เปิด disturbance feedforward (ต้องมี Kalman tau_d) |

### 📊 Status — อ่านอย่างเดียว

| ตัวแปร | คำอธิบาย |
|--------|---------|
| `dev_dash.Status.pos_deg` | ตำแหน่งปัจจุบัน (degrees) |
| `dev_dash.Status.vel_rad_s` | ความเร็วปัจจุบัน (rad/s) |
| `dev_dash.Auto.pos_ideal` | trajectory reference position (rad) |
| `dev_dash.Auto.vel_ideal` | trajectory reference velocity (rad/s) |
| `dev_dash.Auto.pos_err` | position error (rad) |
| `dev_dash.Auto.pwm_out` | final motor command (-1.0 ถึง 1.0) |
| `dev_dash.Auto.traj_active` | 1 = trajectory กำลังทำงาน |
| `dev_dash.In.estop` | raw ESTOP pin state |

---

## ขั้นตอน Homing

```
1. LT + X  (หรือ Modbus 0x01 = 1)
   → STATE_HOMING_FAST: หมุนซ้าย 25% PWM

2. HOME_SENSOR ทริค (PC3 rising edge)
   → STATE_HOMING_BACKOFF: direct PWM ขวา 20% นาน 1s

3. → STATE_HOMING_SLOW: หมุนซ้าย 6% (ช้าที่สุด)

4. HOME_SENSOR ทริคอีกครั้ง
   → finish_homing(): encoder=0, homed=1 → STATE_IDLE

ไฟ: เหลืองกะพริบเร็ว → เหลือง/เขียวสลับ → เหลือง pulse → เขียวติด
```

> **ทดสอบ P2P โดยไม่มี sensor:** Set `dev_dash.Auto.set_home = 1` ใน Live Expressions

---

## การสั่ง P2P (ผ่าน Live Expressions)

```
1. ตรวจว่า current_state == STATE_IDLE หรือ STATE_AUTO
2. ตรวจว่า homed == 1
3. dev_dash.Auto.target_deg = 90.0     ← ใส่เป้าหมาย (degrees)
4. dev_dash.Auto.start_move = 1        ← trigger move
   → เครื่องเข้า STATE_AUTO อัตโนมัติ
5. ดู ss_error_deg เพื่อดู steady-state error
```

---

## State Machine

```
STATE_INIT
  ↓
STATE_IDLE ──────────────────────────────── ← ทุก state กด LB หรือ ESTOP → EMER
  │ LT+X / reg[0x01]=1
  ↓
STATE_HOMING_FAST → left 25%
  │ sensor
  ↓
STATE_HOMING_BACKOFF → right 20% × 1s (direct PWM)
  │ ticks done
  ↓
STATE_HOMING_SLOW → left 6%
  │ sensor → finish_homing()
  ↓
STATE_IDLE (homed=1)
  │ MODE switch LOW
  ↓
STATE_AUTO ← P2P / Modbus / Joystick
  │ reg[0x22] > 0
  ↓
STATE_SEQUENCE ← pick/place หลายจุด

STATE_TEST ← reg[0x01] bit4

STATE_EMER ← ESTOP / LB / cancel_move
  └── ออก: ปล่อย ESTOP + กด RESET ค้าง 50ms
```

---

## Tower Lights (Production Mode)

| สี | รูปแบบ | ความหมาย |
|----|--------|---------|
| 🔴 solid + 🟡 1Hz | EMER — ESTOP กดอยู่ |
| 🔴🟡🟢 blink 2Hz พร้อมกัน | EMER — ปล่อย ESTOP แล้ว → กด RESET |
| 🟡 solid | Not homed (startup) |
| 🟡 5Hz blink | HOMING_FAST |
| 🟡🟢 alternating 2Hz | HOMING_BACKOFF |
| 🟡 1Hz pulse | HOMING_SLOW |
| 🟢 solid | IDLE (homed) |
| 🟢🟡 solid | MANUAL |
| 🟢 solid | AUTO / SEQUENCE |
| 🟢 solid + 🟡 2Hz | TEST |
| + 🔴 5Hz flash | ใกล้ soft limit (±440°) |

---

## Cascade PID Architecture

```
              ideal_pos ─────────────────────────────────────────┐
                                                                  ↓
actual_pos ──→[ − ]──→ Position PID ──→ vel_sp ──→[ + ] ──→[ − ]──→ Velocity PID ──→ PWM%
                                                   ↑              ↑
                                              ideal_vel      vel_filtered
                                                   ↑
                                          RefFF (optional)
```

**Outer loop (Position):**  
`vel_sp = Kp_pos·e_pos + Ki_pos·∫e_pos − Kd_pos·v_filtered`

**Inner loop (Velocity):**  
`pwm% = Kp_vel·e_vel + Ki_vel·∫e_vel + Kd_vel·(Δe_vel/dt)`

Anti-windup: integrator clamped ก่อน accumulate  
D-term velocity: ใช้ `−vel_filtered` ลด noise

---

## File Structure

```
Core/
  Inc/
    main.h            ← pin defines, homing params, CAN node ID
    pid_control.h     ← PID gains extern declarations
    modbus.h          ← register map (MODBUS_REG_COUNT=64)
    joystick.h        ← Xbox packet parser
    REF_FEEDFORWARD.h ← reference feedforward
    DistFF.h          ← disturbance feedforward
  Src/
    main.c            ← state machine, control loop, all logic
    pid_control.c     ← Velocity_PID(), Position_PID() + gains
    stm32g4xx_it.c    ← ISR: TIM7, EXTI3(home), EXTI9_5(mode)
    REF_FEEDFORWARD.c ← model-based voltage FF
    DistFF.c          ← disturbance FF (ต้องมี Kalman tau_d)
    ENCODER.c         ← windowed velocity encoder
    SCURVE.c          ← 5-seg S-curve trajectory
    TRAPEZOID.c       ← 3-seg trapezoid trajectory

docs/
  instruction.html   ← Full Phase 1+2 setup guide (เปิดใน browser)
  PROJECT_CONTEXT.md ← Project context สำหรับ report/AI
```

---

## Modbus Register Map (สรุปสำคัญ)

> FC06 Write / FC03 Read · Slave ID=21 · 19200 baud 8E1

| Addr | R/W | ชื่อ | Format |
|------|-----|------|--------|
| 0x00 | RW | Heartbeat | Robot→PC: 22881 / PC→Robot: 18537 |
| 0x01 | W | Mode | 1=Home, 4=Auto, 8=SetHome, 16=Test |
| 0x0C | W | Kp_vel | uint16 ×100 |
| 0x0D | W | Ki_vel | uint16 ×100 |
| 0x0E | W | Kd_vel | uint16 ×100 |
| 0x0F | W | Apply gains | write 1 → apply |
| 0x24 | W | P2P target | int16 (degrees) |
| 0x25 | W | Soft stop | 1 = หยุดทันที (no EMER) |
| 0x27 | R | Task state | 0=Idle, 1=Homing, 2=Pick, 4=Place |
| 0x28 | R | Position | int16 deg×10 |
| 0x29 | R | Velocity | int16 (deg/s)×10 |
| 0x31 | R | ESTOP | 1=active |
| 0x3D | W | Kp_pos | uint16 ×100 |
| 0x3F | W | Ki_pos | uint16 ×100 |

Full register map อยู่ใน `docs/instruction.html` ส่วน Modbus

---

## ESTOP / EMER Behavior

| เหตุการณ์ | ผล |
|----------|-----|
| ESTOP pin LOW ค้าง 200ms | → STATE_EMER ทันที |
| LB button บน joystick | → STATE_EMER ทันที |
| ออกจาก EMER | ปล่อย ESTOP + กด RESET ค้าง 50ms พร้อมกัน |
| กด RESET แล้วไม่ออก | ESTOP ยังอยู่หรือมี noise → ปล่อยทุกอย่างแล้วกดใหม่ |

---

## Soft Limits

```
MIN_ANGLE_DEG = -540°   (1.5 รอบ ซ้าย)
MAX_ANGLE_DEG = +540°   (1.5 รอบ ขวา)
ถ้าใกล้ ±440° → ไฟแดงกะพริบเตือน
```

Move ที่เกิน limit จะถูก reject ใน `start_move_deg()` ไม่วิ่ง

---

## Tips สำหรับ Control Team

**เริ่ม tune PID:**
1. Set `pid_enabled = 0` เพื่อ disable ก่อน
2. ปรับค่า `kp_vel`, `ki_vel` ใน Live Expressions
3. Set `pid_enabled = 1` กลับ

**ถ้า motor ไม่หมุนแต่ PID กำลัง output:**
- ดู `dev_dash.Auto.pwm_out` ว่ามีค่ามั้ย
- ดู `pid_enabled` ว่าเป็น 1
- ดู `dev_dash.Ctrl.mode` ว่าเป็น 0 (PRODUCTION)

**ถ้า encoder กลับทิศ:**
- Set `encoder_inverted = 1` (ค่า default แล้ว)

**ถ้า motor กลับทิศ:**
- Set `motor_dir_inverted = 1`

**ถ้า integral ล้น (windup):**
- Set `vel_integral = 0` ใน Live Expressions

**ทดสอบ sensor:**
- ดู `home_sensor_raw` ใน Live Expressions — ควรเป็น 0 ปกติ, 1 ตอนมีแม่เหล็ก

**Reference feedforward:**
- Set `V_supply = 24.0` (bus voltage จริง)
- Set `refff_enabled = 1`
- ดูว่า overshoot ลดลงมั้ย
