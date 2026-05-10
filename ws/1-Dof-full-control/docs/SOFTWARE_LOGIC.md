# Software Logic — 1-DOF Robot Control System

---

## Architecture Overview

```
[TIM7 @1kHz ISR]                    [Main Loop while(1)]
  ├─ อ่าน Encoder (TIM1)              └─ if (flag_10ms) @ 100Hz
  ├─ Poll Power Button                    ├─ อ่าน ADC Current (PC4)
  ├─ Modbus timeout tick                  ├─ Modbus process
  └─ ตั้ง flag_10ms ทุก 10 รอบ           ├─ อัพเดท Modbus registers
                                          ├─ เช็ค MODE switch
[EXTI ISR]                               ├─ System Mode → State Machine
  ├─ PB13 (ESTOP) → STATE_EMER           ├─ Apply Motor (ramp + dead-time)
  └─ PB5 (MODE) → Auto/Manual           └─ อัพเดท dev_dash
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

| ปุ่ม/แกน | Mode | Action |
|---------|------|--------|
| LY + RT (กำค้าง) | Manual/Joystick Test | ควบคุมมอเตอร์ (Deadman switch) |
| L3 + R3 | Production | Software E-Stop → STATE_EMER |
| LT + BTN_X | Production | เข้า STATE_CALIBRATE (Homing) |
| BTN_BACK | Production | Reset alarm จาก STATE_EMER |
| BTN_Y | Manual | กระบอกลม ON |
| BTN_X | Manual | กระบอกลม OFF |
| BTN_A | Manual | Pick sequence (ลม+gripper) |
| BTN_B | Manual | Place sequence |
| BTN_LB | Manual/Auto | → STATE_EMER |
| DPAD_UP/DOWN/LEFT | Joystick Test | Tower G/Y/R |
| DPAD_RIGHT | Joystick Test | EMER_OUTPUT |
| BTN_Y/B/X | Joystick Test | Pneumatic/Gripper/Reset LED |

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
// TIM1, Encoder Mode TI12 (4x resolution)
// ใน TIM7 1kHz ISR:
static uint16_t last_cnt = 0;
uint16_t now_cnt = __HAL_TIM_GET_COUNTER(&htim1);
current_position += (int32_t)(int16_t)(now_cnt - last_cnt);
last_cnt = now_cnt;
// current_position เป็น int32_t, รองรับ travel ไม่จำกัด (delta accumulation)
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
