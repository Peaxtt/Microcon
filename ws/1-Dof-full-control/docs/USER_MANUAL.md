# คู่มือการใช้งาน — ระบบหุ่นยนต์หยิบ-วาง 1 แกน (1-DOF Pick & Place Robot)
## FRA263/FRA264 — NUCLEO-G474RE · BaseSystem v1.1

---

## สารบัญ

1. [ภาพรวมระบบ](#1-ภาพรวมระบบ)
2. [ฮาร์ดแวร์](#2-ฮาร์ดแวร์)
3. [การเชื่อมต่อและการเปิดใช้งาน](#3-การเชื่อมต่อและการเปิดใช้งาน)
4. [สถานะระบบและไฟสัญญาณ Tower Light](#4-สถานะระบบและไฟสัญญาณ-tower-light)
5. [โหมดการทำงาน](#5-โหมดการทำงาน)
6. [การใช้งาน BaseSystem](#6-การใช้งาน-basesystem)
7. [การควบคุมด้วย Joystick](#7-การควบคุมด้วย-joystick)
8. [ลำดับการทำงาน Pick & Place (Sequence)](#8-ลำดับการทำงาน-pick--place-sequence)
9. [ระบบ Emergency และการกู้คืน](#9-ระบบ-emergency-และการกู้คืน)
10. [การปรับแต่งค่า Live Expressions](#10-การปรับแต่งค่า-live-expressions)
11. [Modbus Register Map](#11-modbus-register-map)
12. [Python Logger — การวินิจฉัยปัญหา](#12-python-logger--การวินิจฉัยปัญหา)
13. [การ Build และ Flash Firmware](#13-การ-build-และ-flash-firmware)

---

## 1. ภาพรวมระบบ

ระบบนี้เป็น **หุ่นยนต์หยิบ-วางแท่งโลหะ 1 แกน** ที่ควบคุมด้วย STM32G474RE บอร์ด NUCLEO-G474RE

### การทำงาน
- จานหมุนมีรูทุก **5°** (72 ตำแหน่ง รัศมี 0–359°)
- แขนกลหมุนหาตำแหน่งรู **หยิบ** แท่งโลหะออก แล้วหมุนไปวางที่รู **วาง**
- ควบคุมผ่าน PC (BaseSystem .exe), แผงสลับ Cabinet Switch, หรือ Xbox Joystick
- ระบบควบคุม: Cascade PID (Velocity + Position) พร้อม Trajectory Generator

### สถาปัตยกรรม Firmware

```
1 kHz ISR (TIM7):
  ├── อ่าน Encoder → คำนวณตำแหน่ง / ความเร็ว / Cumulative
  ├── Soft Limit: |cumul_deg| ≥ 540° → ล็อกทิศทาง
  └── control_update(pos, vel, &pwm) → ส่ง PWM ไปมอเตอร์

100 Hz Main Loop (flag_10ms):
  ├── อ่าน Sensor (ESTOP, Reed, Home, Mode Switch, ADC)
  ├── รับ Modbus commands จาก BaseSystem
  ├── State Machine → Tower Lights + Actuator
  └── ส่ง Modbus feedback (ตำแหน่ง / ความเร็ว / state)
```

---

## 2. ฮาร์ดแวร์

### บอร์ดและอุปกรณ์หลัก

| อุปกรณ์ | รุ่น / รายละเอียด |
|---------|-----------------|
| Microcontroller | STM32G474RE (NUCLEO-G474RE) |
| มอเตอร์ | DC Motor + Encoder 8192 counts/rev |
| Motor Driver | Cytron MDD3A |
| Joystick | Xbox Controller → RP2040-Zero → USART3 @ 460800 8N1 |
| PC Interface | Modbus RTU (LPUART1, 19200 8E1, Slave ID=21) |

### Pin Assignment

| Pin | สัญญาณ | หมายเหตุ |
|-----|--------|---------|
| PC9 | PWM มอเตอร์ | TIM3 CH4, 20 kHz |
| PC7 | MOTOR_DIR | ทิศทางมอเตอร์ |
| PA8/PA9 | ENC_A / ENC_B | TIM1 Encoder mode |
| PA15 | ESTOP | PULLUP, LOW = กด (active) |
| PC3 | HOME_SENSOR | PULLUP, HIGH = ตรวจพบ |
| PC2 | MODE Switch | PULLUP, LOW = Modbus AUTO, HIGH = MANUAL |
| PC13 | RESET_BTN | PULLUP, LOW = กด |
| PA7 | REED_UP | PULLDOWN, HIGH = active |
| PA4 | REED_DOWN | PULLDOWN, HIGH = active |
| PB0 | REED_GRIP | PULLDOWN, HIGH = active |
| PB1 | PNEUMATIC | OUT — ควบคุมกระบอกลม |
| PB2 | GRIPPER | OUT — ควบคุมคีม |
| PB13/14/15 | TOWER_R/Y/G | ไฟสัญญาณหอ 3 สี |
| PA0 | Current ADC | ADC2 DMA |
| PA2/PA3 | LPUART1 TX/RX | Modbus RS-232/485 |
| PB8/PB9 | USART3 RX/TX | Joystick RP2040 |
| PC11 | EMER_OUTPUT | Relay Fail-safe: HIGH=ปกติ, LOW=ฉุกเฉิน |
| PD2 | RESET_LED | LED ปุ่ม Reset |

---

## 3. การเชื่อมต่อและการเปิดใช้งาน

### ขั้นตอนการเปิดระช้งาน

1. **เชื่อมต่อสาย USB** จาก PC ไปยัง ST-Link ของ NUCLEO-G474RE
2. **เชื่อมต่อสาย RS-232/RS-485** จาก PC ไปยัง LPUART1 (Modbus)
3. **เปิด BaseSystem.exe** บน PC
4. **ตั้งค่า Serial Port**: COM port ที่ตรงกัน, 19200 baud, 8E1, Slave ID=21
5. ตรวจสอบ **Heartbeat** ใน BaseSystem ต้องเป็นสีเขียว (Connected)
6. กดปุ่ม **HOME** เพื่อให้หุ่นยนต์หาจุดอ้างอิง

### Cabinet Switch (PC2)

| สถานะ | โหมด | คำอธิบาย |
|-------|------|---------|
| LOW (ต่อลง GND) | **Modbus AUTO** | BaseSystem ควบคุมได้ทุกคำสั่ง |
| HIGH (ไม่ต่อ / PULLUP) | **Manual Joystick** | Joystick ควบคุมมอเตอร์โดยตรง, BaseSystem ถูก block |

> **หมายเหตุ:** ต้องใช้สาย short PC2 ลง GND เพื่อเข้าโหมด Modbus

---

## 4. สถานะระบบและไฟสัญญาณ Tower Light

### ความหมายของสี

| สถานะระบบ | ไฟแดง (R) | ไฟเหลือง (Y) | ไฟเขียว (G) |
|----------|-----------|------------|------------|
| **EMER** — ESTOP กดอยู่ | กระพริบ 2 Hz | ดับ | ดับ |
| **EMER** — ESTOP ปล่อยแล้ว รอกด RESET | ติดค้าง | ดับ | ดับ |
| **HOMING_FAST** | ดับ | กระพริบ 5 Hz (เร็ว) | ดับ |
| **HOMING_BACKOFF** | ดับ | กระพริบ 2.5 Hz | ดับ |
| **HOMING_SLOW** | ดับ | กระพริบ 1 Hz (ช้า) | ดับ |
| **IDLE** — ยังไม่ Home | ดับ | Y↔G สลับ 2 Hz | Y↔G สลับ 2 Hz |
| **IDLE** — Home แล้ว | ดับ | ดับ | ติดค้าง |
| **IDLE** — Set Home ใหม่ | ดับ | Y-Y-G Flash | G Flash |
| **MANUAL** (Cabinet switch) | ดับ | ติดค้าง | ดับ |
| **MANUAL_MB** (BaseSystem) | ดับ | ติดค้าง | ดับ |
| **AUTO** — เคลื่อนที่เร็ว (>4 rad/s) | ดับ | ดับ | กระพริบ 5 Hz |
| **AUTO** — เคลื่อนที่ปานกลาง | ดับ | ดับ | กระพริบ 1.7 Hz |
| **AUTO** — เคลื่อนที่ช้า | ดับ | ดับ | กระพริบช้า |
| **AUTO** — หยุดนิ่ง (Settled) | ดับ | ดับ | ติดค้าง |
| **SEQUENCE** | ดับ | ติดค้าง | กระพริบ 2 Hz |
| **TEST** | ดับ | กระพริบ 2 Hz | ติดค้าง |
| **Soft Limit** (±540°) | ติดค้าง | ดับ | ติดค้าง |

### Reset LED (PD2)

| สถานะ | ความหมาย |
|-------|---------|
| กระพริบ | อยู่ใน EMER + ESTOP ปล่อยแล้ว → รอกดปุ่ม RESET |
| ดับ | ระบบปกติ |

---

## 5. โหมดการทำงาน

### State Enum (reg 0x2F)

```
INIT=0 → IDLE=1
                ├── HOME → HOMING_FAST(2) → HOMING_BACKOFF(3) → HOMING_SLOW(4) → IDLE
                ├── MANUAL(cmd 0x02) → MANUAL_MB(6)
                ├── AUTO(cmd 0x04, ต้อง Home ก่อน) → AUTO(7)
                │         └── SEQ_START → SEQUENCE(8)
                └── TEST(cmd 0x10, ต้อง Home) → TEST(9)
MANUAL_SWITCH(5) ← Cabinet switch HIGH (ทุก state)
EMER(10)         ← ESTOP LOW / Joystick X (ทุก state)
```

### ลำดับการเริ่มใช้งาน (ทั่วไป)

```
เปิดเครื่อง → IDLE → [กด HOME] → Homing... → IDLE (homed)
→ [กด MANUAL หรือ AUTO ใน BaseSystem] → ทำงานตามต้องการ
```

---

## 6. การใช้งาน BaseSystem

### แท็บ HOME

| ปุ่ม / ฟิลด์ | คำอธิบาย |
|------------|---------|
| **HOME** | เริ่ม Homing Sequence (Fast → Backoff → Slow) |
| **SET HOME** | กำหนดตำแหน่งปัจจุบันเป็นจุด Home (Virtual Zero) |
| Status: Homed | 1 = Home สำเร็จ, 0 = ยังไม่ได้ Home |

### แท็บ MANUAL

| ปุ่ม / ฟิลด์ | คำอธิบาย |
|------------|---------|
| **JOG +N°** | หมุนตามเข็มนาฬิกา N องศา จากตำแหน่งปัจจุบัน |
| **JOG −N°** | หมุนทวนเข็มนาฬิกา N องศา |
| **UP / DOWN** | ขยับกระบอกลมขึ้น / ลง |
| **OPEN / CLOSE** | เปิด / ปิดคีม |
| **PICK** | Sequence หยิบ (ลง → หนีบ → ขึ้น) |
| **PLACE** | Sequence วาง (ลง → ปล่อย → ขึ้น) |

### แท็บ AUTO

| ปุ่ม / ฟิลด์ | คำอธิบาย |
|------------|---------|
| **Target (°)** | ตำแหน่งเป้าหมาย (องศา, 0–360) |
| **GO** | สั่งเคลื่อนที่ไปยัง Target ด้วย Trajectory Control |
| **STOP** | หยุดการเคลื่อนที่ทันที (Soft Stop, ค้างตำแหน่ง) |
| **JOG** | Jog สัมพัทธ์จากตำแหน่งปัจจุบัน |
| Status: Moving | 1 = กำลังเคลื่อนที่, 0 = หยุดแล้ว |

> **หมายเหตุ:** ต้องกด HOME ก่อนเข้าแท็บ AUTO

### แท็บ SEQUENCE

| ฟิลด์ | คำอธิบาย |
|-------|---------|
| **Pair Count** | จำนวน Pick-Place คู่ (สูงสุด 8 คู่) |
| **Pick [1..8]** | รหัสรูสำหรับหยิบ (hole index, 0–71, × 5° = องศา) |
| **Place [1..8]** | รหัสรูสำหรับวาง |
| **START** | เริ่ม Sequence อัตโนมัติ |
| **STOP** | หยุด Sequence |

### แท็บ TEST

ใช้สำหรับทดสอบการเคลื่อนที่ซ้ำ ๆ ระหว่าง 2 ตำแหน่ง
- กำหนด Start Position, End Position, จำนวนรอบ
- ต้อง Home ก่อน

---

## 7. การควบคุมด้วย Joystick

> ใช้เมื่อ Cabinet Switch อยู่ใน HIGH (MANUAL) หรือ ระบบ connect Joystick

### Mapping (Xbox Controller)

| ปุ่ม / Axis | การทำงาน |
|------------|---------|
| **RT (ค้าง) + LY** | ควบคุมมอเตอร์ (deadman) สูงสุด ±60% PWM พร้อม Ramp |
| **DPAD ←** | Fine tune หมุนซ้าย ±`homing_slow_speed` (default 7%) |
| **DPAD →** | Fine tune หมุนขวา |
| **Y (กดครั้งเดียว)** | Toggle กระบอกลม (ขึ้น ↔ ลง) |
| **B (กดครั้งเดียว)** | Toggle คีม (เปิด ↔ ปิด) |
| **LB (กดครั้งเดียว)** | Pick Sequence (เปิด → ลง → หนีบ → ขึ้น) |
| **RB (กดครั้งเดียว)** | Place Sequence (ลง → ปล่อย → ขึ้น) |
| **RT (ค้าง) + A** | Set Home ณ ตำแหน่งปัจจุบัน |
| **X** | เข้า EMERGENCY ทันที |

### Dead Zone

- Stick dead zone: ±8% (จาก ±0.08)
- Ramp rate: 3% ต่อ tick (100 Hz) → ~200ms ถึงความเร็วสูงสุด

---

## 8. ลำดับการทำงาน Pick & Place (Sequence)

### ลำดับ PICK (1 คู่)

```
1. GOING_PICK  : เปิดคีม (ขณะเคลื่อนที่) → หมุนไปตำแหน่ง Pick
2. GOING_PICK  : รอ ctrl_settled (ถึงตำแหน่ง) → ส่ง Cylinder ลง
3. PICKING-0   : รอ reed_down + dwell → ปิดคีม (Grip)
4. PICKING-1   : รอ reed_grip + dwell → ส่ง Cylinder ขึ้น
5. PICKING-2   : รอ reed_up + dwell  → เข้า GOING_PLACE
6. GOING_PLACE : หมุนไปตำแหน่ง Place (คีมถือแท่งอยู่)
7. GOING_PLACE : รอ ctrl_settled → ส่ง Cylinder ลง
8. PLACING-0   : รอ reed_down + dwell → เปิดคีม (Release)
9. PLACING-1   : รอ !reed_grip + dwell → ส่ง Cylinder ขึ้น
10.PLACING-2   : รอ reed_up + dwell  → คู่ถัดไป / เสร็จสิ้น
```

### Reed Switch Logic

| reed_dummy_en | การทำงาน |
|--------------|---------|
| **1 (เปิด)** | `reed_down/up/grip` อัพเดตตาม Output GPIO หลังจาก `reed_dummy_delay_ms` (30ms) |
| **0 (ปิด)** | อ่านจาก Hardware จริง + Dwell 200ms ก่อนยืนยัน |

```
SEQ_OK = (timer ≥ seq_dwell_ms  AND  reed = true)
       OR (timer ≥ seq_timeout_ms)
```

### ค่าที่ปรับได้

| ตัวแปร | Default | ความหมาย |
|--------|---------|---------|
| `seq_dwell_ms` | 200 ms | เวลาขั้นต่ำก่อนยืนยัน reed |
| `seq_timeout_ms` | 3000 ms | บังคับเดินต่อถ้าไม่มี reed |
| `reed_dummy_en` | 1 | 1=Dummy mode, 0=Real hardware |
| `reed_dummy_delay_ms` | 30 ms | Delay จำลอง reed |

---

## 9. ระบบ Emergency และการกู้คืน

### การเข้า EMERGENCY

ระบบเข้า EMERGENCY เมื่อ:
1. **ESTOP ถูกกด** (PA15 = LOW) นาน 200 ms ขึ้นไป
2. **Joystick กดปุ่ม X**
3. **Soft Limit ±540°** — ไม่เข้า EMER แต่ล็อกทิศทาง

เมื่อเข้า EMERGENCY:
- มอเตอร์หยุดทันที (PWM = 0)
- Relay EMER_OUTPUT = LOW (de-energize)
- ไฟแดงสว่าง / กระพริบ

### การออกจาก EMERGENCY

```
1. ปล่อย ESTOP (PA15 กลับเป็น HIGH)
2. กดปุ่ม RESET ค้าง 50 ms
3. ระบบกลับสู่ STATE_IDLE
4. Relay EMER_OUTPUT = HIGH (re-energize)
```

> **หมายเหตุ:** ถ้าออกจาก EMER แล้ว Cabinet Switch ยัง HIGH → กลับไป STATE_MANUAL

### Soft Rotation Limit (±540°)

- ระบบติดตาม `cumulative_angle_deg` (องศาสะสมจาก Home)
- เมื่อถึง ±540° (1.5 รอบ): ล็อกทิศทางนั้น ยังหมุนได้ทิศตรงข้าม
- ไฟ Tower: R + G ติดค้างพร้อมกัน
- เพื่อปลดล็อก: หมุนกลับเข้ามา หรือ Set Home ใหม่

### Fail-Safe Relay

| สถานะ | EMER_OUTPUT |
|-------|------------|
| ระบบปกติ (boot, idle, run) | **HIGH** (Relay ดึงอยู่) |
| EMERGENCY active | **LOW** (Relay ปล่อย) |

---

## 10. การปรับแต่งค่า Live Expressions

เปิด STM32CubeIDE → Debug → Live Expressions → พิมพ์ชื่อตัวแปร

### dev_dash.Ctrl (Control Team Interface)

| Section | ตัวแปร | Default | หน่วย |
|---------|--------|---------|-------|
| **Trajectory** | `dev_dash.Ctrl.max_velocity` | 3.0 | rad/s |
| | `dev_dash.Ctrl.max_accel` | 12.56 | rad/s² |
| | `dev_dash.Ctrl.max_jerk` | 10.0 | rad/s³ |
| | `dev_dash.Ctrl.traj_type` | 0 | 0=Trap, 1=S-Curve |
| **Position PID** | `dev_dash.Ctrl.kp_position` | 2.0 | — |
| | `dev_dash.Ctrl.ki_pos` | 0.7 | — |
| | `dev_dash.Ctrl.kd_pos` | 0.2 | — |
| | `dev_dash.Ctrl.pos_deadband_deg` | 2.0 | ° |
| | `dev_dash.Ctrl.vel_deadband` | 1.0 | rad/s |
| **Velocity PID** | `dev_dash.Ctrl.kp_vel` | 10.0 | — |
| | `dev_dash.Ctrl.ki_vel` | 0.01 | — |
| | `dev_dash.Ctrl.kd_vel` | 0.0 | — |
| **Feedforward** | `dev_dash.Ctrl.refff_enabled` | 1 | 0/1 |
| | `dev_dash.Ctrl.V_supply` | 24.0 | V |
| **Hardware** | `dev_dash.Ctrl.motor_dir_inverted` | 1 | 0/1 |

### Commands ผ่าน dev_dash.Ctrl

| ตัวแปร | ค่า | การทำงาน |
|--------|-----|---------|
| `dev_dash.Ctrl.start_move` | 1 | เริ่มเคลื่อนที่ไป `traj_target_deg` (auto-clear) |
| `dev_dash.Ctrl.reset_all` | 1 | รีเซ็ต PID integrators + หยุดการเคลื่อนที่ |
| `dev_dash.Ctrl.zero_encoder` | 1 | Set Home ณ ตำแหน่งปัจจุบัน |

### Observables (R/O)

| ตัวแปร | ความหมาย |
|--------|---------|
| `dev_dash.Ctrl.ss_error_deg` | ค่าความผิดพลาดตำแหน่ง (°) |
| `dev_dash.Ctrl.ss_reached` | 1 = ถึงเป้าหมายแล้ว (Settled) |
| `dev_dash.Ctrl.vel_error_live` | ค่าความผิดพลาดความเร็ว |
| `dev_dash.Ctrl.g_smooth_vel` | ความเร็วที่ผ่าน Filter (deg/s) |
| `dev_dash.Ctrl.g_pwm_duty` | PWM output (−1 ถึง +1) |
| `dev_dash.Ctrl.g_current_A` | กระแสมอเตอร์ (A) |

### ค่า Homing (pรับขณะ Debug)

| ตัวแปร | Default | ความหมาย |
|--------|---------|---------|
| `homing_fast_speed` | 0.25 | PWM Fast sweep (25%) |
| `homing_backoff_speed` | 0.20 | PWM Backoff (20%) |
| `homing_slow_speed` | 0.06 | PWM Slow approach (6%) — ใช้ร่วมกับ DPAD fine tune |

### ค่า Sequence

| ตัวแปร | Default | ความหมาย |
|--------|---------|---------|
| `seq_dwell_ms` | 200 | ms ขั้นต่ำก่อน accept reed |
| `seq_timeout_ms` | 3000 | ms บังคับเดินต่อ |
| `reed_dummy_en` | 1 | 1=Dummy, 0=Real |
| `reed_dummy_delay_ms` | 30 | Delay จำลอง reed (ms) |

---

## 11. Modbus Register Map

**Protocol:** Modbus RTU · LPUART1 · 19200 8E1 · Slave ID = 21 · FC03 (Read) / FC06 (Write)

### Command Registers (BaseSystem → Firmware)

| Register | ชื่อ | ค่า | คำอธิบาย |
|---------|------|-----|---------|
| 0x00 | Heartbeat | PING=18537, PONG=22881 | Connection monitor |
| 0x01 | Mode Command | bit0=HOME, bit1=MANUAL, bit2=AUTO, bit3=SET_HOME, bit4=TEST | One-hot, auto-clear |
| 0x02 | Actuator | 0x00=UP, 0x01=DOWN, 0x02=OPEN, 0x04=CLOSE | Edge-detect |
| 0x03 | Manual Seq | bit0=Pick, bit1=Place | Edge-detect |
| 0x04 | Seq Start | 0x01 = Start Sequence | Auto-clear |
| 0x05 | Jog | signed int16 (degrees) | + = CW, − = CCW |
| 0x24 | P2P Target | signed int16 (degrees) | ใช้เมื่อ AUTO |
| 0x25 | Soft Stop | 0x01 = หยุด | Auto-clear |

### Sequence Registers

| Register | คำอธิบาย |
|---------|---------|
| 0x22 | จำนวน pair (0–8) |
| 0x12+i×2 | Pick position pair i (hole index × 5° = degrees) |
| 0x13+i×2 | Place position pair i |

### Feedback Registers (Firmware → BaseSystem)

| Register | ชื่อ | หน่วย / ค่า |
|---------|------|-----------|
| 0x23 | Homed | 0 = ยังไม่ Home, 1 = Homed |
| 0x26 | Reed switches | bit0=UP, bit1=DOWN, bit2=GRIP |
| 0x27 | Status | bit0=Homing, bit1=Picking, bit2=Placing, bit3=Moving |
| 0x28 | Position | 0–3599 (× 0.1° = 0–359.9°) |
| 0x29 | Velocity | signed (deg/s × 10) |
| 0x30 | Acceleration | signed (deg/s² × 10) |
| 0x2F | State Enum | ดูตาราง State |
| 0x31 | ESTOP | 0 = ปกติ, 1 = กด |
| 0x32 | Digital IO | bit0=HOME_SENSOR, bit2=RESET_BTN, bit3=MODE |
| 0x3A | Current | mA |

### State Enum (reg 0x2F)

| ค่า | State | ความหมาย |
|-----|-------|---------|
| 0 | INIT | เริ่มต้นระบบ |
| 1 | IDLE | รอคำสั่ง |
| 2 | HOMING_FAST | Homing ระยะแรก (เร็ว) |
| 3 | HOMING_BACKOFF | ถอยหลังหลัง sensor |
| 4 | HOMING_SLOW | Homing ระยะสุดท้าย (ช้า) |
| 5 | MANUAL_SWITCH | Cabinet switch HIGH — Joystick |
| 6 | MANUAL_MB | BaseSystem MANUAL tab |
| 7 | AUTO | BaseSystem AUTO tab — PID control |
| 8 | SEQUENCE | Pick & Place อัตโนมัติ |
| 9 | TEST | Test Mode |
| 10 | EMER | Emergency |

---

## 12. Python Logger — การวินิจฉัยปัญหา

### robot_logger.py

**วัตถุประสงค์:** แสดงกราฟ Real-Time Position, Velocity, และ Tracking Diagnosis

```bash
cd C:\PaYae\Microcon\ws\1-Dof-full-control\docs
python robot_logger.py COM3
```

**ต้องการ:** `pip install pyserial matplotlib`

### กราฟที่แสดง

| กราฟ | ความหมาย |
|-----|---------|
| Position + Target | ตำแหน่งปัจจุบัน vs เป้าหมาย (°) |
| Velocity | ความเร็วเชิงมุม (deg/s) |
| Tracking Diagnosis | วิเคราะห์ error vs velocity |

### การอ่าน Tracking Diagnosis

| สี Background | ความหมาย | การแก้ไข |
|-------------|---------|---------|
| **เขียว** | TRACKING — velocity วิ่งหา target ✓ | ปกติ |
| **แดง** | INVERTED — velocity วิ่งออกจาก target | ตรวจ `motor_dir_inverted` หรือ `encoder_inverted` |
| เทา | ข้อมูลไม่เพียงพอ / หยุดอยู่ | — |

### python_gui (S-Curve Tuner)

**วัตถุประสงค์:** ปรับจูน PID และ Trajectory แบบ Real-Time ผ่าน PyQt5

```bash
cd C:\PaYae\Microcon\ws\1-Dof-full-control\python_gui
pip install -r requirements.txt
python main.py
```

> อ่าน Telemetry Protocol และ Modbus Regs 10–25 (ชุด Python GUI แยกจาก BaseSystem)

---

## 13. การ Build และ Flash Firmware

### Build ด้วย STM32CubeIDE

1. เปิดโปรเจค `1-Dof-full-control` ใน STM32CubeIDE
2. Refresh Project: คลิกขวา → **Refresh (F5)** เพื่อให้ IDE รับรู้ไฟล์ใหม่
3. **Build**: `Ctrl+B`
4. **Flash & Debug**: `F11`

### ไฟล์ที่ต้องอยู่ใน CubeIDE Source Tree

| ไฟล์ | บทบาท |
|------|-------|
| `Core/Src/main.c` | State machine, ISR, Modbus, ทุกอย่าง |
| `Core/Src/control.c` | Cascade PID + Trajectory (Control Team) |
| `Core/Src/pid_control.c` | PID variables / legacy gains |
| `Core/Src/ENCODER.c` | Encoder driver |
| `Core/Src/SCURVE.c` | S-Curve trajectory generator |
| `Core/Src/TRAPEZOID.c` | Trapezoid trajectory generator |
| `Core/Src/REF_FEEDFORWARD.c` | Reference feedforward model |
| `Core/Src/DistFF.c` | Disturbance feedforward |
| `Core/Src/joystick.c` | Xbox joystick parser (RP2040) |
| `Core/Src/modbus.c` | Modbus RTU driver |

### หลังแก้ไขค่า Default

- แก้ใน `main.c` → initializer `DevDashboard_t dev_dash = { ... }`
- แก้ใน `control.c` → `ctrl_pos_deadband_deg`, `ctrl_vel_deadband`
- Build + Flash ใหม่ ค่าจะติดถาวร

---

## ภาคผนวก — Troubleshooting

| อาการ | สาเหตุ | วิธีแก้ |
|-------|-------|--------|
| BaseSystem กดไรไม่ได้ | Cabinet switch HIGH (PC2 float) | Short PC2 ลง GND |
| มอเตอร์หมุนไม่หยุด | encoder/motor direction ผิด | ดู Tracking Diagnosis, ตั้ง `motor_dir_inverted=1` |
| Jog หมุนผิดทาง | Sign convention | ตรวจสอบว่า CCW command → CCW motion |
| Homing ไม่มีแรง | PWM ต่ำเกินไป | เพิ่ม `homing_fast_speed` |
| ค่า settled กระพริบ | Velocity noise > deadband | เพิ่ม `dev_dash.Ctrl.vel_deadband` |
| Sequence ไม่วิ่งถึงก่อน | traj_done race *(แก้แล้ว)* | Build ใหม่ |
| Pick/Place ผิดตำแหน่ง | Hole index ไม่ × 5 *(แก้แล้ว)* | Build ใหม่ |
| motor_dir_inverted reset เป็น 0 | Sync จาก dev_dash.Ctrl | แก้ที่ `dev_dash.Ctrl.motor_dir_inverted` |

---

*คู่มือนี้อ้างอิงจาก Firmware commit หลัง session 2026-06-01*  
*FRA263/FRA264 — FIBO KMUTT*
