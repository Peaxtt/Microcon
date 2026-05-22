# 1-DOF Robot Control System — Project Context
> ใช้สำหรับเขียนรายงาน / ส่งให้ AI เป็น context  
> Last updated: 2026-05-22

---

## ภาพรวมระบบ

ระบบควบคุมหุ่นยนต์ 1 องศาอิสระ (1-DOF) สำหรับงาน Pick-and-Place  
พัฒนาบน **NUCLEO-G474RE** (dev board) เป้าหมายย้ายไป **custom PCB** หลัง fabricate เสร็จ

| รายการ | รายละเอียด |
|--------|-----------|
| MCU | STM32G474RETx @ 170 MHz |
| Dev board | NUCLEO-G474RE (LQFP64) |
| Prod board | Custom PCB (ออกแบบแล้ว รอ fabricate) |
| Motor driver | Cytron MDD3A (DIR + PWM) |
| Encoder | Quadrature, 8192 counts/rev (TIM1 TI12) |
| Joystick | Xbox controller → RP2040-Zero (USB-C) → USART3 460800 8N1 |
| PC interface | Modbus RTU FC06, LPUART1, 19200 8E1, Slave ID=21 |
| Software | BaseSystem (FRA263/264) — compiled .exe ฝั่ง PC |

---

## Software Architecture

### Control Loop — TIM7 @ 1 kHz (ISR)
```
Encoder → Windowed velocity → IIR filter → Cascade PID → PWM
                                           ↑
                                   Trajectory Generator
                                (Trapezoid / S-Curve)
```

**Cascade PID:**
- Outer loop: Position PID → velocity setpoint  
  `vel_sp = Kp_pos·e_pos + Ki_pos·∫e_pos - Kd_pos·v_filtered + v_ideal`
- Inner loop: Velocity PID → PWM%  
  `pwm = Kp_vel·e_vel + Ki_vel·∫e_vel + Kd_vel·(Δe_vel/dt)`
- Anti-windup: accumulator clamping (ไม่ clamp output)
- D-term ใช้ `-velocity` feedback ไม่ใช่ `d(error)/dt`

**Default PID gains (pid_control.c):** kp_vel=30, ki_vel=0.1, kd_vel=0.0, kp_pos=1.0

**Trajectory Generators:**
- S-Curve (5-segment constant-jerk) — `SCURVE.h`
- Trapezoid (3-segment) — `TRAPEZOID.h`

**Encoder:**
- Windowed velocity: 10-sample ring buffer (`VEL_WINDOW=10`)
- `ENCODER.h` → `Encoder_t` struct

### Main Loop — flag_10ms @ 100 Hz
- ESTOP polling (debounce 200ms anti-EMI)
- Reed switch read
- ADC current sensor (moving average 1/8 IIR)
- Modbus process
- State machine execution
- Tower light state machine
- Live Expressions dashboard update

---

## State Machine

```
STATE_INIT
  ↓ boot
STATE_IDLE  ←────────────────────────────────────────────────┐
  │ LT+X (joy) / Modbus reg01 bit0                           │
  ↓                                                           │
STATE_HOMING_FAST   → motor left 25% PWM                     │
  │ sensor EXTI                                               │
  ↓                                                           │
STATE_HOMING_BACKOFF → direct PWM 20% right 1s                │
  │ traj_done + settled                                       │
  ↓                                                           │
STATE_HOMING_SLOW   → motor left 1.3% PWM                    │
  │ sensor EXTI → finish_homing()                            │
  ↓                                                           │
STATE_IDLE (homed=1)                                         │
  │ MODE=LOW → STATE_AUTO                                     │
  │ Modbus 0x02 / joy / P2P                                   │
STATE_AUTO ─────────────────────────────────────────────────┘
  │ MODE=HIGH
STATE_MANUAL

STATE_EMER  ← ESTOP pin LOW / LB joy / cancel_move
  └── ออก: ปล่อย ESTOP + กด RESET ค้าง
```

**Homing detail (Two-Pass):**
1. `STATE_HOMING_FAST`: หมุนซ้าย 25% PWM
2. Rising edge (HOME_SENSOR PC3, PULLUP active HIGH) → ISR EXTI3
3. `STATE_HOMING_BACKOFF`: direct PWM ขวา 20% นาน 100 ticks (1s)
4. `STATE_HOMING_SLOW`: หมุนซ้าย 6% PWM
5. Rising edge อีกครั้ง → `finish_homing()` → `cumulative_angle_deg=0`, `homed=1`

**Soft limits:** ±540° (1.5 รอบ) จาก home — check ใน `start_move_deg()`

---

## Tower Light Logic (Production Mode)

| State | Green | Yellow | Red |
|-------|-------|--------|-----|
| EMER | off | 1 Hz blink | solid |
| Not homed | off | steady | off |
| HOMING_FAST | off | 5 Hz blink | off |
| HOMING_BACKOFF | alt 2 Hz | alt 2 Hz | off |
| HOMING_SLOW | off | 1 Hz pulse (8%) | off |
| IDLE (homed) | steady | off | off |
| MANUAL | steady | steady | off |
| AUTO moving | 4 Hz blink | off | off |
| AUTO settled | steady | off | off |
| AUTO + Test | steady | 1 Hz blink | off |

---

## Pin Assignments (Current NUCLEO-G474RE)

### Motor & Encoder
| Pin | Function | Config |
|-----|----------|--------|
| PA0 | MOTOR_DIR | Output PP |
| PA6 | PWM (TIM3 CH1) | AF2, 20kHz (period=8499) |
| PA8 | ENC_A (TIM1 CH1) | AF6 |
| PA9 | ENC_B (TIM1 CH2) | AF6 |

### Sensors & Safety
| Pin | Function | Config | Logic |
|-----|----------|--------|-------|
| PC3 | HOME_SENSOR | PULLUP, EXTI3 rising | HIGH=detected |
| PA1 | REED_UP | PULLDOWN | LOW=triggered |
| PA4 | REED_DOWN | PULLDOWN | LOW=triggered |
| PB0 | REED_GRIP | PULLDOWN | LOW=triggered |
| PB13 | ESTOP | PULLUP | LOW=active |
| PC13 | RESET_BTN | PULLUP | LOW=active |
| PB10 | POWER_BTN | PULLUP | LOW=active |
| PB5 | MODE | PULLUP, EXTI | LOW=AUTO |

### Actuators & Indicators
| Pin | Function |
|-----|----------|
| PC1 | POWER_LATCH (cylinder DOWN) |
| PB11 | GRIPPER solenoid |
| PC6 | PNEUMATIC solenoid |
| PC7 | TOWER_G |
| PB7 | TOWER_Y |
| PC8 | TOWER_R |
| PB14 | EMER_OUTPUT relay |
| PB4 | RESET_LED |
| PA5 | LD2 (onboard heartbeat) |

### Communication
| Pin | Peripheral | Config | Function |
|-----|-----------|--------|----------|
| PA2 | LPUART1_TX | 19200 8E1 | Modbus → PC |
| PA3 | LPUART1_RX | 19200 8E1 | Modbus ← PC |
| PC10 | USART3_TX | 460800 8N1 | → RP2040 |
| PC11 | USART3_RX | 460800 8N1 | ← RP2040 |
| PA11 | FDCAN1_RX | AF9 | CAN Bus |
| PA12 | FDCAN1_TX | AF9 | CAN Bus |
| PC4 | ADC2_IN5 | Analog | Current sensor |

---

## Modbus Register Map (Slave ID=21, 19200 8E1)

### PC → Robot (Write)
| Addr | Name | Encoding |
|------|------|----------|
| 0x00 | Heartbeat reply | PC writes 18537 ("HI") |
| 0x01 | Mode | 1=Home, 2=Jog, 4=Auto, 8=SetHome, 16=Test |
| 0x02 | Gripper manual | bit0=Down, bit1=Open, bit2=Close |
| 0x03 | Sequence trigger | bit0=Pick, bit1=Place (edge-triggered) |
| 0x04 | Gripper enable | bit0: enable auto gripper |
| 0x05 | Jog | int16, degrees |
| 0x06 | Test type | 0=Precision, 1=Performance |
| 0x07 | Perf speed | int16 deg/s |
| 0x08 | Perf accel | int16 deg/s² |
| 0x09 | Prec start | int16, degrees |
| 0x10 | Prec end | int16, degrees |
| 0x11 | Prec repeats | uint16 |
| 0x12–0x21 | Seq slots ×10 | int16, degrees (pick/place pairs) |
| 0x22 | Pair count | uint16 |
| 0x23 | P2P unit | 0=degree, 1=index |
| 0x24 | P2P target | int16 |
| 0x25 | Soft stop | bit0: 1=stop |

### Robot → PC (Read)
| Addr | Name | Encoding |
|------|------|----------|
| 0x00 | Heartbeat | robot writes 22881 ("YA") |
| 0x26 | Reed | bit0=Up, bit1=Down, bit2=Grip |
| 0x27 | Task | 0=Idle, 1=Homing, 2=Pick, 4=Place, 8=GoPoint |
| 0x28 | Position | int16, degrees × 10 |
| 0x29 | Velocity | int16, deg/s × 10 |
| 0x30 | Acceleration | int16, deg/s² × 10 |
| 0x31 | ESTOP | 1=active |

### Custom (Python GUI / Live Expressions)
| Addr | Name |
|------|------|
| 0x0C–0x0F | Kp/Ki/Kd_vel ×100 + apply trigger |
| 0x38 | Traj type (0=SCurve, 1=Trapezoid) |
| 0x39 | v_max ×100 |
| 0x3A | Current mA (read only) |
| 0x3B | a_max ×100 |
| 0x3C | j_max ×100 |
| 0x3D–0x3F | Kp/Kd/Ki_pos ×100 |

---

## CAN Bus (FDCAN1)

- Transceiver: TJA1050 (on PCB, internal trace PA11/PA12)
- Rate: 1 Mbit/s classic CAN
- Config: Prescaler=5, TimeSeg1=25, TimeSeg2=8, SJW=1
- Node ID: `CAN_NODE_ID = 0x001`
- Filter: accept-all → FIFO0

---

## New PCB Pin Changes (Phase 2)

| Signal | Current (NUCLEO) | New PCB |
|--------|-----------------|---------|
| HOME_SENSOR | PC3 | PC3 (ไม่เปลี่ยน) |
| PWM | PA6 (TIM3 CH1) | PC9 (TIM3 CH4) |
| TOWER_G | PC7 | PB12 |
| POWER_BTN | PB10 | PB2 |
| RESET_LED | PB4 | PB1 |
| MODE | PB5 | PC2 |

> IOC ต้องอัพเดทก่อน regen สำหรับ Phase 2

---

## Key Files
| File | Description |
|------|-------------|
| `Core/Src/main.c` | Main firmware — all logic |
| `Core/Inc/main.h` | Pin defines + homing/CAN defines |
| `Core/Src/stm32g4xx_it.c` | ISR handlers (EXTI9_5, TIM7, UART) |
| `Core/Inc/joystick.h` | Xbox controller packet parser |
| `Core/Inc/modbus.h` | Modbus RTU slave |
| `Core/Src/SCURVE.c` | 5-segment S-curve trajectory |
| `Core/Src/TRAPEZOID.c` | 3-segment trapezoid trajectory |
| `Core/Src/ENCODER.c` | Windowed velocity encoder |
| `docs/homing_spec.md` | Homing system spec (source of truth) |
| `docs/PCB_WIRING.md` | PCB terminal layout |
| `docs/PCB_ROUTING.md` | NUCLEO → terminal routing guide |
| `docs/pin_out.md` | Full pin reference + Modbus map |
| `docs/instruction.html` | Setup & operation guide (Phase 1+2) |
