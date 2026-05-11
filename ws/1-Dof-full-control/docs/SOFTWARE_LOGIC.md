# Software Logic — 1-DOF Robot Control System

---

## Architecture Overview

```
[TIM7 @1kHz ISR]                         [Main Loop while(1)]
  ├─ Encoder delta → current_position       └─ if (flag_10ms) @ 100Hz
  ├─ Windowed velocity → ctrl_vel_rad_s         ├─ ADC Current (PC4)
  ├─ Acceleration LPF → ctrl_acc_rad_s2         ├─ Modbus process
  ├─ [STATE_AUTO] Trajectory update             ├─ Modbus registers
  ├─ [STATE_AUTO] Position PID                  ├─ MODE switch check
  ├─ [STATE_AUTO] Velocity PID → PWM            ├─ State Machine (triggers)
  ├─ Poll Power Button                          ├─ [Non-AUTO] Motor Apply
  └─ Modbus timeout tick                        ├─ Send_Telemetry() → UART4
                                                ├─ dev_dash update
[EXTI ISR]                                      └─ IWDG Refresh
  ├─ PB13 (ESTOP) → STATE_EMER
  └─ PB5 (MODE) → Auto/Manual
```

---

## State Machine (5 States)

```
         ┌─────────────────────────────────────┐
         ▼                                     │
   [STATE_IDLE]                                │
    motor=0, รอคำสั่ง                          │
    │  │  │                                    │
    │  │  └─ Modbus 0x01|0x04 ──► [STATE_AUTO] │
    │  └─── Modbus 0x01|0x02 ──► [STATE_MANUAL]│
    └────── Modbus 0x01|0x01 ──► [STATE_CALIBRATE]
    
   [STATE_MANUAL]                              │
    จอย LY + RT deadman → motor_speed_cmd      │
    BTN_LB → [STATE_EMER]                      │
    RT ปล่อย + LY=0 → [STATE_IDLE]            │
                                               │
   [STATE_CALIBRATE]                           │
    motor_speed_cmd = 0.1 (ช้า)               │
    HOME sensor → [STATE_IDLE]  ← ยังไม่ implement│
                                               │
   [STATE_AUTO]                                │
    รอ Control Team ใส่ PID                    │
    BTN_LB → [STATE_EMER]                      │
                                               │
   [STATE_EMER] ◄──── PB13 (EXTI, ทันที) ─────┘
    motor=0, TOWER_R=ON, EMER_OUTPUT=HIGH
    ออกได้: คลาย E-Stop + กด BTN_BACK (จอย) หรือ Modbus 0x01=0xFF
```

---

## Motor Control Pipeline

```
Joystick/Modbus/State Machine
         │
         ▼
  motor_speed_cmd  (-1.0 ถึง +1.0)
         │
         ▼
  [Direction Change Dead-time]  ← 50ms hold ที่ 0 ก่อนสลับทิศ
         │
         ▼
  [Slew Rate Limiter]  ← dev_dash.Ctrl.ramp_rate per 10ms
         │
         ▼
  [Max Speed Cap]  ← dev_dash.Ctrl.max_speed (0.0-1.0)
         │
         ├─► DIR pin (PA0): + = GPIO_SET, - = GPIO_RESET
         │
         └─► PWM (TIM3_CH1 PA6): CCR = |speed| × ARR(8499)
```

---

## System Modes (dev_dash.Ctrl.mode)

| Mode | ค่า | ชื่อ | คำอธิบาย |
|------|-----|------|---------|
| Production | 0 | `SYS_MODE_PRODUCTION` | รัน State Machine ปกติ |
| Hardware Test | 1 | `SYS_MODE_HARDWARE_TEST` | บังคับ Output โดยตรงจาก dev_dash.Ctrl |
| Joystick Test | 2 | `SYS_MODE_JOYSTICK_TEST` | จอยควบคุม Output และมอเตอร์ตรงๆ |
| Auto Motor Test | 3 | `SYS_MODE_AUTO_MOTOR_TEST` | สลับทิศอัตโนมัติ, ปรับ speed/period ได้ |

---

## Joystick Mapping (XInput, USART3 460800 8N1)

### Mode 2 — Joystick Test (bypass State Machine, ทดสอบ hardware ตรงๆ)
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
| RT (กำค้าง) + LY | Motor speed | PA6 PWM |

### Mode 0 — Production (State Machine)
| ปุ่ม/แกน | Action |
|---------|--------|
| LT + BTN_X | เข้า STATE_CALIBRATE (Homing) |
| RT + LY | เข้า STATE_MANUAL + ควบคุมมอเตอร์ |
| L3 + R3 | Software E-Stop → STATE_EMER |
| BTN_LB | → STATE_EMER (ใน Manual/Auto) |
| BTN_BACK | Reset alarm จาก STATE_EMER |
| BTN_X (Manual) | กระบอกลม OFF |
| BTN_Y (Manual) | กระบอกลม ON |
| BTN_A (Manual) | Pick sequence |
| BTN_B (Manual) | Place sequence |

Joystick timeout: ถ้าไม่ได้รับ packet >500ms → `connected=0` → motor=0

---

## Current Sensor Logic

```c
// ADC2_IN5 (PC4), WCS1800 @ 5V supply
float v   = (filtered_adc / 4095.0f) * 3.3f;       // ADC → Voltage
float i_a = (v - cur_zero_v) / cur_sens;             // Voltage → Amperes
// cur_zero_v = 2.5V (VCC/2 ที่ 5V supply)
// cur_sens   = 0.066 V/A (66mV/A)
// EMA filter: alpha = 1/8 (7×old + 1×new) / 8
```

ผล: `dev_dash.Status.current_A` เป็น float หน่วย **Amperes, ทศนิยม 4 ตำแหน่ง**

---

## Encoder Logic

```c
// TIM1, Encoder Mode TI12 (4x resolution) — 2048 PPR × 4 = 8192 counts/rev
// ใน TIM7 1kHz ISR:
static uint16_t enc_last = 0;
uint16_t enc_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
int32_t enc_delta = (int32_t)(int16_t)(enc_now - enc_last);
enc_last = enc_now;
current_position += enc_delta;  // int32_t, ไม่ overflow

// Windowed velocity (10-sample ring buffer @ 1kHz):
// ctrl_vel_rad_s = sum(10 deltas) × (2π / 8192 / 10 / 0.001)
// หน่วย: rad/s, lag ~5ms, noise ≈ 0.077 rad/s
```

**Unit conversions:**
- `RAD_PER_CNT = 2π / 8192 ≈ 7.67×10⁻⁴ rad/count`
- `pos_rad = current_position × RAD_PER_CNT`
- `vel_rad_s` = windowed velocity จาก ISR (global: `ctrl_vel_rad_s`)

---

## Telemetry — Simulink

ส่งทุก 10ms (100Hz) จาก main loop — **16 bytes** per packet:

```
Byte  0-1 : 7E 7E              ← header (2 bytes)
Byte  2-5 : position  float32  ← rad    (4 bytes)
Byte  6-9 : velocity  float32  ← rad/s  (4 bytes)
Byte 10-13: acceleration float32 ← rad/s² (4 bytes)
Byte 14-15: 03 03              ← terminator (2 bytes)
```
Total: **16 bytes** per packet (ไม่มี checksum)

**เลือก UART ผ่าน Live Expressions:**

| `telemetry_mode` | UART | Baud | COM Port | Modbus |
|-----------------|------|------|---------|--------|
| `0` (default) | UART4 (PB9/PB8) | 115200 | USB-UART adapter | ✅ ใช้ได้ปกติ |
| `1` | LPUART1 (USB ST-Link) | 19200 | ST-Link VCP | ❌ ปิด |

**ตั้งค่าที่ Live Expressions:**
```
dev_dash.Ctrl.telemetry_mode = 1   ← สลับมา USB / ปิด Modbus
dev_dash.Ctrl.telemetry_mode = 0   ← กลับ Modbus ปกติ
```

**ฝั่ง Simulink:**
```
Serial Receive (ระบุ COM port + baud ตามตาราง, 16 bytes)
  → Byte Unpack
      bytes [2:5]   → float → position (rad)
      bytes [6:9]   → float → velocity (rad/s)
      bytes [10:13] → float → acceleration (rad/s²)
```

---

## Modbus Register Map (Slave ID: 21, LPUART1 19200 8E1)

### WRITE (PC → STM32)
| Register | ความหมาย | ค่า |
|---------|---------|-----|
| 0x00 | Heartbeat RX | ส่ง 18537 เพื่อ reply |
| 0x01 | Mode Command | 0x01=Home, 0x02=Manual, 0x04=Auto, 0xFF=Reset alarm, 0x10=Test mode |
| 0x03 | Output Control | bit0=Pneumatic, bit1=Gripper, bit2=TowerG, bit3=TowerY |
| 0x05 | Jog Command | (int16_t) ความเร็ว Jog |
| 0x24 | P2P Target | (int16_t) ตำแหน่งเป้าหมาย |
| 0x25 | Stop Command | 0x01=Soft stop |

### READ (STM32 → PC)
| Register | ความหมาย | ค่า |
|---------|---------|-----|
| 0x00 | Heartbeat TX | STM32 ส่ง 22881 |
| 0x04 | Current (mA) | กระแสมอเตอร์ (uint16, mA) |
| 0x26 | Reed Sensors | bit0=Up, bit1=Down, bit2=Grip |
| 0x27 | Current Task | 0=Idle/Manual, 1=Homing, 8=P2P |
| 0x28 | Encoder Position | (int16_t) ตำแหน่ง encoder |
| 0x31 | Emergency Status | 1=Active |
