# 1-DOF Industrial Robot Control System

> **STM32G474RETx** · Cytron MD20A · 24VDC / 750W · Modbus RTU + XInput Joystick

---

## ภาพรวมระบบ

```
┌─────────────────────────────────────────────────────────────┐
│                    CONTROL CABINET                          │
│                                                             │
│  [PC — Base System UI]                                      │
│       ↕  Modbus RTU  (LPUART1, 19200 8E1, Slave ID: 21)    │
│  [STM32G474RE — Firmware]                                   │
│       ├─ TIM7 @1kHz  → Encoder · Power Button · Modbus     │
│       ├─ Loop @100Hz → ADC Current · State Machine · Motor  │
│       ├─ EXTI        → E-Stop (PB13) · Mode Switch (PB5)   │
│       └─ USART3      → Joystick XInput (460800 8N1)         │
│              ↕ PWM 20kHz + DIR                              │
│       [Cytron MD20A 20A] → [DC Motor + Encoder]            │
│                                                             │
│  Sensors: WCS1800 Current · Reed Up/Down/Grip              │
│  Outputs: Tower Light · Pneumatic · Gripper · EMER         │
└─────────────────────────────────────────────────────────────┘
```

**สถานะปัจจุบัน**
- ✅ Infrastructure (UART DMA, Timer, State Machine, Modbus, Joystick)
- ✅ Motor: หมุน Forward/Reverse ผ่าน Joystick และ Auto Test
- ✅ Current Sensor: WCS1800 บน PC4, หน่วย Amperes
- ✅ Encoder: TI12 mode, absolute position tracking
- ⬜ Homing / Calibrate: ยังไม่ implement (HOME sensor ยังไม่มีขา)
- ⬜ PID Position Control: รอทีม Control

---

## อ่านอะไรก่อน — แล้วแต่ทีม

### 🔧 ทีม Control (ใส่ PID / Homing)
```
1. README.md          ← อยู่นี่แล้ว — ภาพรวมและ key variables
2. DEVELOPER_GUIDE.md ← Live Expressions, สิ่งที่ต้อง implement, การ tuning
3. SOFTWARE_LOGIC.md  ← State machine ละเอียด, Motor pipeline, Modbus map
4. HARDWARE_SETUP.md  ← Pin mapping ครบ, ถ้าต้องการ HOME sensor
```

### 🖥️ ทีม UI / Base System
```
1. README.md          ← ภาพรวม + Modbus map สรุป (ด้านล่าง)
2. SOFTWARE_LOGIC.md  ← Modbus register map ครบถ้วน
```

### 🤖 AI / Developer คนถัดไป
```
1. docs/AI_HANDOFF_CONTEXT.md  ← อ่านอันเดียวได้ครบทุกอย่าง
```

### 🧪 ทีม Testing / Commissioning
```
1. README.md         ← ภาพรวม
2. TESTING_GUIDE.md  ← ขั้นตอนทดสอบ Mode 1-3, Checklist
```

---

## โครงสร้างไฟล์

```
1-Dof-full-control/
│
├── README.md                    ← คุณอยู่ที่นี่ — อ่านก่อน
│
├── 1-Dof-full-control.ioc       ← CubeMX config (source of truth pin mapping)
│
├── Core/
│   ├── Inc/
│   │   ├── main.h              ← Pin label defines ทั้งหมด
│   │   └── joystick.h          ← Joystick API + button defines
│   └── Src/
│       ├── main.c              ← โค้ดหลัก: State Machine, Motor, ADC, Dashboard
│       ├── joystick.c          ← Parse XInput 15-byte packet, deadzone
│       ├── modbus.c            ← Modbus RTU slave, DMA, register R/W
│       ├── stm32g4xx_hal_msp.c ← GPIO/Clock/DMA (CubeMX generated)
│       └── stm32g4xx_it.c      ← Interrupt handlers (CubeMX generated)
│
└── docs/
    ├── AI_HANDOFF_CONTEXT.md   ← Context ครบสำหรับ developer คนถัดไป
    ├── HARDWARE_SETUP.md       ← Pinout ทุกขา, Timer, Power, Current Sensor
    ├── SOFTWARE_LOGIC.md       ← State machine, Motor pipeline, Modbus map
    ├── DEVELOPER_GUIDE.md      ← Live Expressions, Test modes, API variables
    └── TESTING_GUIDE.md        ← ขั้นตอนทดสอบ, Calibration, Checklist
```

---

## Key Variables (Control Team ใช้)

```c
// Motor
extern float   motor_speed_cmd;    // เขียน: -1.0 ถึง +1.0

// Encoder & Motion (อัพเดท @1kHz ใน TIM7 ISR)
extern int32_t current_position;   // counts, TI12 4x (8192 counts/rev)
extern float   ctrl_vel_rad_s;     // velocity (rad/s), windowed 10ms
extern float   ctrl_acc_rad_s2;    // acceleration (rad/s²), LPF

// Current (อัพเดท @100Hz)
extern float   current_sensor_A;   // Amperes
```

**Unit:** `pos_rad = current_position × (2π/8192)`

**Pipeline (Non-AUTO):**
```
motor_speed_cmd → [Max Speed Cap] → [Dead-time 50ms] → [Slew Ramp] → PWM
```

**STATE_AUTO:** PID รันใน TIM7 ISR @1kHz → SET_COMPARE โดยตรง (ข้าม slew)

---

## Live Expressions — เปิดทุกอย่างด้วย 1 expression

```
Flash ด้วย Debug (🐞) → Live Expressions → Add: dev_dash
```

```
▼ dev_dash
  ▼ Ctrl           ← ปรับ real-time ได้ทั้งหมด
      mode                  0=Production · 1=HW Test · 2=Joy Test · 3=Auto Test
      ramp_rate             0.03   (slew rate /10ms)
      max_speed             0.40   (hard cap 0.0–1.0)
      auto_speed            0.30   (speed ใน mode 3)
      auto_period_fwd_ms    1000   (ms ทิศ Forward)
      auto_period_rev_ms    1000   (ms ทิศ Reverse)
      cur_zero_v            2.50   (zero-current voltage, วัดจาก multimeter)
      cur_sens              0.066  (sensitivity V/A)
      force_motor_speed     0.0    (ใช้ใน mode 1)
      force_pneumatic/gripper/tower_*  (ใช้ใน mode 1)
  ▼ Status         ← อ่าน real-time
      state                 STATE_IDLE / MANUAL / AUTO / EMER
      motor_cmd             speed จริงที่ออก (signed)
      encoder               ตำแหน่ง encoder (int32, counts)
      current_A             กระแส (Amperes)
      pos_rad               ตำแหน่ง (rad) — ตรงกับที่ส่งไป Simulink
      vel_rad_s             ความเร็ว (rad/s)
      acc_rad_s2            ความเร่ง (rad/s²)
  ▼ In             ← สถานะปุ่มและ switch
      estop / mode_switch / reset / power
  ▼ Out            ← สถานะ output จริง
      pwm/dir/pneumatic/gripper/tower_g/y/r
  ▼ Auto           ← STATE_AUTO trajectory + PID (เขียน/อ่าน real-time)
      target_deg            เป้าหมาย (องศา)
      start_move            set 1 เพื่อ trigger move
      traj_type             0=Trapezoid · 1=S-Curve · 2=Direct PID (ไม่มี trajectory)
      time_mode             0=constraint-based · 1=time-based
      v_max / a_max / j_max ← trajectory limits
      kp_vel / ki_vel / kd_vel ← velocity PID gains
      kp_pos / ki_pos / kd_pos ← position PID gains
      — อ่าน status —
      pos_rad / vel_rad_s   ← feedback (rad, rad/s)
      pos_ideal / vel_ideal ← trajectory reference
      pos_err / vel_sp      ← error / velocity setpoint
      pwm_out               ← final motor command
      traj_active           ← 1=กำลังวิ่ง
```

---

## Pin Mapping (สรุป)

| Pin | Label | หน้าที่ |
|-----|-------|--------|
| **PA6** | MOTOR_PWM | TIM3_CH1 · PWM 20kHz → Cytron MD20A |
| **PA0** | MOTOR_DIR | HIGH=Forward, LOW=Reverse |
| **PA8** | ENCODER_B | TIM1_CH1 · Encoder TI12 |
| **PA9** | ENCODER_A | TIM1_CH2 · Encoder TI12 |
| **PC4** | CURRENT_SENSOR | ADC2_IN5 · WCS1800 (VCC=5V) |
| **PB13** | ESTOP | EXTI Falling → STATE_EMER ทันที (PULLUP override ใน code) |
| **PB5** | MODE | EXTI · Auto/Manual selector (PULLUP override ใน code) |
| **PA1/PA4/PB0** | REED_UP/DOWN/GRIP | Digital Input Pull-up |
| **PC6/PB11** | PNEUMATIC/GRIPPER | Digital Output |
| **PC7/PC8/PB7** | TOWER G/R/Y | Tower Light |
| **PB14** | EMER_OUTPUT | HIGH เมื่อ E-Stop active |
| **PB6** | POWER_LATCH | ล็อคไฟ (SET=ON ตอน startup, ปัจจุบัน comment ออก) |
| **PA2/PA3** | LPUART1 TX/RX | Modbus RTU 19200 8E1 |
| **PC10/PC11** | USART3 TX/RX | Joystick 460800 8N1 |
| **PB9/PB8** | UART4 TX/RX | Telemetry → Simulink 115200 (ต้องใช้ adapter) |
| **LPUART1** | USB ST-Link | Telemetry → Simulink 19200 เมื่อ `telemetry_mode=1` |

> Pin mapping ครบถ้วน → [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)

---

## Modbus Register Map (สรุป)

**Slave ID: 21 · 19200 8E1 · LPUART1**

### WRITE (PC → STM32)
| Register | ความหมาย | ค่า |
|---------|---------|-----|
| `0x00` | Heartbeat | ส่ง `18537` เมื่อเห็น `22881` |
| `0x01` | Mode Command | `1`=Home · `2`=Manual · `4`=Auto · `0xFF`=Reset alarm |
| `0x03` | Output Control | bit0=Pneumatic · bit1=Gripper · bit2=TowerG · bit3=TowerY |
| `0x05` | Jog | (int16) ความเร็ว jog |
| `0x24` | P2P Target | (int16) ตำแหน่งเป้าหมาย (encoder counts) |
| `0x25` | Soft Stop | `1` = หยุด |

### READ (STM32 → PC)
| Register | ความหมาย | ค่า |
|---------|---------|-----|
| `0x00` | Heartbeat | STM32 ส่ง `22881` |
| `0x04` | Current | กระแส (mA, uint16) |
| `0x26` | Reed Sensors | bit0=Up · bit1=Down · bit2=Grip |
| `0x27` | Current Task | `0`=Idle · `1`=Homing · `8`=P2P |
| `0x28` | Encoder Position | (int16) encoder counts |
| `0x31` | Emergency | `1`=Active |

> Modbus ละเอียดครบ → [docs/SOFTWARE_LOGIC.md](docs/SOFTWARE_LOGIC.md)

---

## สิ่งที่ต้องทำต่อ

| งาน | ใคร | ไฟล์ |
|-----|-----|------|
| กำหนดขา HOME sensor ใน .ioc | Hardware | HARDWARE_SETUP.md |
| Implement Homing logic | Control | DEVELOPER_GUIDE.md |
| Implement PID (STATE_AUTO) | Control | DEVELOPER_GUIDE.md |
| ใส่ Capacitor 4700µF×3 บน MD20A VIN | Hardware | — |
| ตั้ง ADC Sampling 47.5 cycles ใน .ioc | Firmware | ทุกครั้งที่ Generate Code |
| แก้ TIM3 เป็น "PWM Generation CH1" ใน .ioc | Firmware | HARDWARE_SETUP.md |

---

## Base System — วิธีเปิด UI

```bash
docker load -i frontend-image.tar
docker-compose up -d
# เปิด http://localhost:3000
```

```bash
# รัน Python bridge
main.exe
# ควรเห็น: WebSocket Server is running on ws://localhost:8765...
```

**COM Port (Windows):** Device Manager → Ports → *STMicroelectronics STLink Virtual COM Port (COMx)*
