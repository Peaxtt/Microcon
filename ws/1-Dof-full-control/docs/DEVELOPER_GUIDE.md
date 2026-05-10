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

## สิ่งที่ Control Team ต้อง Implement

### 1. STATE_CALIBRATE (Homing)
```c
case STATE_CALIBRATE:
    motor_speed_cmd = 0.1f; // เคลื่อนไปหา Home sensor
    // เมื่อ HOME sensor ตรวจจับ:
    //   __HAL_TIM_SET_COUNTER(&htim1, 32768); // reset encoder ไปกลาง range
    //   current_position = 0;
    //   current_state = STATE_IDLE;
    break;
```

### 2. STATE_AUTO (PID Position Control)
```c
case STATE_AUTO:
    // ใส่ PID ที่นี่
    int32_t error = target_position - current_position;
    motor_speed_cmd = Kp * error + Ki * integral + Kd * derivative;
    // Clamp: motor_speed_cmd อยู่ใน -1.0 ถึง +1.0
    break;
```

### 3. Global Variables ที่ใช้ได้
```c
extern float    motor_speed_cmd;   // สั่งความเร็ว (-1.0 ถึง +1.0)
extern int32_t  current_position;  // ตำแหน่ง encoder (อัพเดททุก 1ms)
extern float    current_sensor_A;  // กระแส (Amperes, อัพเดททุก 10ms)
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
