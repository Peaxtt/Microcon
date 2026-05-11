# Testing Guide — 1-DOF Robot Control System

---

## เครื่องมือที่ใช้

- **STM32CubeIDE** — Flash + Debug + Live Expressions
- **Multimeter** — วัด voltage สำหรับ calibrate current sensor
- **Joystick** — XInput controller ผ่าน USART3

---

## ขั้นตอนเปิดระบบ

1. เสียบสาย ST-Link (USB) จาก PC ไปยัง Nucleo
2. เปิด Power Supply (24V)
3. Flash โค้ดผ่าน CubeIDE หรือกด Reset บนบอร์ด
4. กด **Debug (🐞)** เพื่อเปิด Live Expressions
5. Add `dev_dash` ใน Live Expressions

---

## Test 1: Hardware Test (Mode 1)

ตรวจสอบ output ทุกตัวโดยไม่ใช้จอย

```
dev_dash.Ctrl.mode = 1
```

| ทดสอบ | ตั้งค่า | ผลที่คาดหวัง |
|------|--------|------------|
| Pneumatic | `force_pneumatic = 1` | กระบอกลมขยับ |
| Gripper | `force_gripper = 1` | Gripper ปิด |
| Tower Green | `force_tower_green = 1` | ไฟเขียวติด |
| Tower Yellow | `force_tower_yellow = 1` | ไฟเหลืองติด |
| Tower Red | `force_tower_red = 1` | ไฟแดงติด |
| EMER Output | `force_emer_out = 1` | Relay EMER ดึง |
| Motor FWD 20% | `force_motor_speed = 0.20` | มอเตอร์หมุน Forward |
| Motor REV 20% | `force_motor_speed = -0.20` | มอเตอร์หมุน Reverse |

**⚠️ เริ่มที่ max_speed ต่ำก่อน** — เพิ่มทีละ 0.05 พร้อมดู current_A

---

## Test 2: Joystick Test (Mode 2)

```
dev_dash.Ctrl.mode = 2
```

| ทดสอบ | วิธี | ผลที่คาดหวัง |
|------|-----|------------|
| Motor FWD | RT (กำ) + LY ดัน | มอเตอร์เร่งช้าๆ (ramp) |
| Motor REV | RT (กำ) + LY ดึง | มอเตอร์สลับทิศ (dead-time 50ms ก่อน) |
| หยุด | ปล่อย RT | มอเตอร์หยุด (ramp ลง) |
| Tower | DPAD UP/DOWN/LEFT | G/Y/R ติด |
| EMER | DPAD RIGHT | EMER_OUTPUT HIGH |
| Pneumatic | BTN_Y | กระบอกลม |
| Gripper | BTN_B | Gripper |

---

## Test 3: Auto Motor Test (Mode 3)

```
dev_dash.Ctrl.mode = 3
dev_dash.Ctrl.auto_speed = 0.20      // เริ่มต่ำๆก่อน
dev_dash.Ctrl.auto_period_fwd_ms = 2000
dev_dash.Ctrl.auto_period_rev_ms = 2000
```

**ดู:** `dev_dash.Status.motor_cmd` สลับ +/- ทุก 2 วินาที
**ดู:** `dev_dash.Status.encoder` เปลี่ยนทิศตามมอเตอร์
**ดู:** `dev_dash.Status.current_A` ไม่เกิน ~15A

---

## Test 4: E-Stop

1. เดิน motor ด้วย mode ใดก็ได้
2. กด E-Stop button (PB13)
3. **ผลที่คาดหวัง:**
   - PWM = 0 ทันที (ISR response)
   - TOWER_R = ON
   - EMER_OUTPUT = HIGH
   - `dev_dash.Status.state = STATE_EMER`
4. คลาย E-Stop + กด BTN_BACK บนจอย → กลับ IDLE

---

## Test 5: Current Sensor Calibration

1. ไม่ต้องเดิน motor (ไม่มีกระแส)
2. ดู `dev_dash.Status.current_A` → ควรใกล้ 0.0000
3. ถ้าไม่ใช่ 0: วัด Multimeter ที่ OUT pin WCS1800
4. ตั้ง `dev_dash.Ctrl.cur_zero_v = <ค่าที่วัด>`
5. รัน motor ที่ current ที่วัดได้จาก Clamp meter
6. ตรวจสอบ `current_A` ตรงกับ Clamp meter หรือไม่
7. ถ้าไม่ตรง ปรับ `cur_sens` (ค่า default = 0.066)

---

## Checklist ก่อนส่งต่อทีม Control

- [ ] Mode 1: Output ทุกตัวทำงานถูกต้อง
- [ ] Mode 2: มอเตอร์หมุน Forward/Reverse ผ่าน Joystick
- [ ] Mode 3: สลับทิศอัตโนมัติ ไม่ตัด PSU
- [ ] E-Stop: ตัดทันที, reset ได้
- [ ] Encoder: `current_position` เปลี่ยนถูกทิศ (บวก=Forward, ลบ=Reverse)
- [ ] Current: `current_A` อ่านค่าสมเหตุสมผล
- [ ] max_speed ที่ทดสอบผ่านแล้ว: ______
- [ ] HOME Sensor: ยังไม่ได้ assign ขา → แจ้งทีม Hardware

---

## การ Tuning สำหรับทีม Control

**ตั้งค่าผ่าน Live Expressions (ไม่ต้อง flash ใหม่):**

| ต้องการ | ปรับที่ | ทิศทาง |
|--------|--------|--------|
| Motor เร่งเร็วขึ้น | `ramp_rate` ↑ | เพิ่ม (max 0.1) |
| PSU ตัดน้อยลง | `ramp_rate` ↓ | ลด |
| ลด max current | `max_speed` ↓ | ลด |
| Current อ่านเพี้ยน | `cur_zero_v` | วัด Multimeter ใหม่ |
| Encoder นับผิดทิศ | สลับสาย A/B บน encoder | Hardware fix |

---

---

# คู่มือทีม Control — ทดสอบ Auto Position Control

> **อ่านอันนี้อันเดียว ไม่ต้องรู้โค้ดข้างใน**
> ทุกอย่างทำผ่าน Live Expressions ใน STM32CubeIDE — ไม่ต้อง flash ใหม่เลย

---

## ก่อนเริ่ม — สิ่งที่ต้องมี

| สิ่งที่ต้องการ | รายละเอียด |
|-------------|---------|
| STM32CubeIDE | ติดตั้งบน PC แล้ว |
| สาย USB | เสียบจาก PC → Nucleo (port ข้างเล็ก ฝั่ง ST-Link) |
| Power Supply | เปิด 24V แล้ว |
| บอร์ด + Motor | เชื่อมต่อครบ |

---

## ขั้นที่ 1 — เปิดระบบ

**1.1 เปิด CubeIDE และ Flash โค้ด**

```
1. เปิด STM32CubeIDE
2. เปิด project: 1-Dof-full-control
3. กดปุ่ม Debug (🐞) บน toolbar
4. รอ flash เสร็จ จะหยุดที่ main()
5. กด Resume (▶) เพื่อรันโปรแกรม
   → LED สีเขียวบนบอร์ดจะกะพริบ = ระบบทำงานแล้ว
```

**1.2 เปิด Live Expressions**

```
1. เมนู Window → Show View → Live Expressions
2. กดปุ่ม + ใน Live Expressions panel
3. พิมพ์:  dev_dash  แล้วกด Enter
4. กดลูกศร ▶ หน้า dev_dash เพื่อ expand
```

> **Live Expressions คืออะไร?**
> เป็น panel ที่แสดงค่าตัวแปรแบบ real-time และแก้ค่าได้ทันที
> ดับเบิ้ลคลิกที่ค่าตัวเลข → พิมพ์ค่าใหม่ → Enter = มีผลทันที ไม่ต้อง flash

---

## ขั้นที่ 2 — ตรวจ Encoder (สำคัญมาก ทำก่อนทุกอย่าง)

PID ขึ้นอยู่กับ encoder — ถ้าอ่านผิดทิศ motor จะวิ่งหนีแทนที่จะเข้าหาเป้า

**2.1 ดูค่า encoder**

ใน Live Expressions กด expand `Status` แล้วดูค่า `encoder`

**2.2 ทดสอบทิศ**

```
หมุน shaft มอเตอร์ด้วยมือ → Forward
ดูค่า encoder ใน Live Expressions:
  ✅ ค่าเพิ่มขึ้น (บวก) = ถูกต้อง
  ❌ ค่าลดลง (ลบ) = สลับสาย A/B บน encoder แล้วเทสใหม่
```

**2.3 ตรวจ velocity**

```
หมุน shaft ช้าๆ ดูค่า Auto → vel_rad_s:
  ✅ มีค่า ≠ 0 และเครื่องหมายถูกต้อง = พร้อมใช้
  ❌ ติดอยู่ที่ 0 ตลอด = แจ้งทีม Hardware
```

---

## ขั้นที่ 3 — เข้า STATE_AUTO

**ดู state ปัจจุบัน:** Live Expressions → `Status` → `state`

กดสวิตช์ MODE บนบอร์ดให้ `state` เป็น `4` (STATE_AUTO)

```
state = 1 → IDLE
state = 3 → MANUAL
state = 4 → AUTO  ← ต้องการอันนี้
state = 5 → EMER  (มีปัญหา → คลาย E-Stop)
```

> ถ้าไม่เปลี่ยน ให้แจ้งทีม Firmware ว่าสวิตช์ MODE อยู่ตำแหน่งไหน

---

## ขั้นที่ 4 — ตั้งค่าเริ่มต้น (ทำครั้งเดียว)

เปิด `Ctrl` และ `Auto` ใน Live Expressions แล้วตั้งค่าเหล่านี้:

```
Ctrl → max_speed    = 0.30     ← จำกัดความเร็วสูงสุด 30% ก่อน (safety)

Auto → traj_type    = 2        ← Direct PID (ไม่มี trajectory เริ่มง่ายสุด)
Auto → kp_pos       = 1.0      ← position gain
Auto → ki_pos       = 0.0
Auto → kp_vel       = 5.0      ← velocity gain (เริ่มต่ำไว้ก่อน)
Auto → ki_vel       = 0.1
Auto → kd_vel       = 0.0
```

> **ดับเบิ้ลคลิก** ที่ตัวเลขในคอลัมน์ Value → พิมพ์ค่าใหม่ → Enter

---

## ขั้นที่ 5 — สั่ง Move แรก

```
Auto → target_deg = 45        ← เป้าหมาย 45 องศา
Auto → start_move = 1         ← กด GO (จะ clear เป็น 0 เอง)
```

**สังเกตขณะวิ่ง:**

| ดูที่ | ความหมาย | ค่าที่ดี |
|------|---------|---------|
| `Auto → traj_active` | 1=กำลังวิ่ง, 0=ถึงแล้ว | เปลี่ยนจาก 1 → 0 |
| `Auto → pos_err` | error (rad) | ลดลงเรื่อยๆ จนใกล้ 0 |
| `Auto → pwm_out` | PWM ออกจริง | ค่อยๆ ขึ้นแล้วลง ไม่กระโดด |
| `Status → current_A` | กระแสมอเตอร์ | ไม่เกิน 10A |

**ผลที่คาดหวัง:** motor หมุนไปหยุดที่ 45° แล้ว `pos_err` ใกล้ 0

---

## ขั้นที่ 6 — Tune Gain (ทำทีละขั้น อย่าเปลี่ยนหลายค่าพร้อมกัน)

### 6.1 ปรับ kp_vel — ให้ตอบสนองเร็วขึ้น

```
ค่าเริ่ม: kp_vel = 5.0
สั่ง: target_deg = 45 → start_move = 1 → ดูผล
เพิ่มเป็น: kp_vel = 10 → target_deg = 0 → start_move = 1 → ดูผล
เพิ่มเป็น: kp_vel = 15 → ...

หยุดเพิ่มเมื่อ: motor เริ่มสั่น / current เกิน 10A
```

| อาการ | สาเหตุ | แก้ |
|------|-------|-----|
| Motor สั่นหรือ oscillate | kp_vel สูงเกิน | ลด kp_vel ลง 2-3 |
| Motor ช้ามาก ไม่ถึงเป้า | kp_vel ต่ำเกิน | เพิ่ม kp_vel |
| ถึงเป้าแล้วแต่มี error เล็กน้อย | ต้องการ ki_vel | ดู 6.2 |

### 6.2 ปรับ ki_vel — กำจัด error ค้าง

```
ถ้า pos_err ไม่ = 0 หลังหยุด:
ki_vel = 0.3 → ลองใหม่
ki_vel = 0.5 → ลองใหม่
ki_vel = 1.0 → ถ้ายังมี

ระวัง: ki_vel มากเกินทำให้ oscillate ช้าๆ หลังหยุด
```

### 6.3 ปรับ kp_pos — ความแม่นยำ

```
kp_pos = 1.0  (default, ใช้ได้เลยส่วนใหญ่)
kp_pos = 2.0  ถ้าต้องการ tracking ดีขึ้น แต่ overshoot มากขึ้น
```

**บันทึก gain ที่ดีที่สุดไว้** ก่อนไปขั้นถัดไป

---

## ขั้นที่ 7 — สั่ง Move ซ้ำ / เปลี่ยนเป้า

```
target_deg = 0  → start_move = 1   (กลับ 0°)
target_deg = 90 → start_move = 1   (ไป 90°)
target_deg = 45 → start_move = 1   (ไป 45°)
```

**ยกเลิกระหว่างวิ่ง:**
```
Auto → cancel_move = 1    ← หยุดทันที กลับ IDLE
```
หลัง cancel ถ้าจะ move ใหม่ ต้องเข้า STATE_AUTO ใหม่ก่อน (กด MODE switch)

**รีเซ็ต encoder ให้เป็น 0 ณ ตำแหน่งปัจจุบัน:**
```
Auto → reset_encoder = 1  ← ตำแหน่งปัจจุบัน = 0 องศา (auto-clear)
```
ใช้เมื่อ: ต้องการกำหนด "จุดเริ่ม" ใหม่โดยไม่ต้อง home

---

## ขั้นที่ 8 — เพิ่ม Trajectory (ทำหลังจาก gain ดีแล้ว)

ใช้ gain จากขั้นที่ 6 แล้วเปลี่ยน traj_type เพื่อให้ motion smooth ขึ้น

**Trapezoidal (เร่ง-คงที่-ชะลอ):**
```
Auto → traj_type = 0
Auto → v_max     = 1.0   ← ความเร็วสูงสุด rad/s (เริ่มช้าๆ)
Auto → a_max     = 3.14  ← acceleration rad/s²

target_deg = 90 → start_move = 1
```

ดูเพิ่ม: `Auto → vel_ideal` ควรเป็น trapezoid shape, `Auto → pos_ideal` กับ `pos_rad` ควรใกล้กัน

เพิ่ม `v_max` ทีละ 0.5 จนได้ความเร็วที่ต้องการ

**S-Curve (smooth ที่สุด):**
```
Auto → traj_type = 1
Auto → j_max     = 10.0  ← jerk limit rad/s³
```

---

## ⚠️ เกิดปัญหา — ทำอะไร

| อาการ | ทำอะไร |
|------|--------|
| Motor วิ่งไม่หยุด / วิ่งออก | `Ctrl → max_speed = 0` ทันที |
| current_A เกิน 15A | กด **E-Stop** (ปุ่มแดงบนตู้ไฟ) |
| Motor สั่นรุนแรง | กด E-Stop หรือ `max_speed = 0` |
| ไฟแดงบน Tower Light ติด | = EMER state → คลาย E-Stop → กด BTN_BACK บนจอย |
| state ไม่เปลี่ยนเป็น AUTO | ลอง MODE switch อีกครั้ง หรือแจ้งทีม Firmware |
| pos_err ไม่ลด / motor วิ่งหนี | หยุดทันที → ตรวจทิศ encoder (ขั้นที่ 2) |

---

## Checklist ก่อนบอกว่า Done

```
□ Encoder: หมุน FWD → encoder บวก ✓
□ Encoder: หมุน 1 รอบ = ±8192 counts ✓
□ vel_rad_s: เปลี่ยนขณะหมุน ≠ 0 ✓

□ Direct PID (traj_type=2):
  □ สั่ง 45° → ถึง pos_err ≈ 0 ✓
  □ สั่ง 0° → กลับ pos_err ≈ 0 ✓
  □ สั่งซ้ำหลายรอบ → stable ✓
  □ gain ที่ใช้: kp_vel=___ ki_vel=___ kp_pos=___

□ Trapezoidal (traj_type=0):
  □ motion smooth ไม่กระตุก ✓
  □ traj_active = 0 เมื่อถึงเป้า ✓
  □ v_max ที่ใช้ได้: ___ rad/s

□ current_A ขณะวิ่ง ไม่เกิน 10A ✓
□ cancel_move ทำงาน (หยุดได้ทุกเมื่อ) ✓
```
