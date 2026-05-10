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
