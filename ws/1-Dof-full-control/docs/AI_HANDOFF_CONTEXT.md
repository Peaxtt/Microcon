# AI Handoff Context — 1-DOF Robot Control System

เอกสารนี้ให้ AI ตัวถัดไปอ่านเพื่อเข้าใจระบบอย่างสมบูรณ์ก่อนเริ่มพัฒนา

---

## 1. Project Overview

- **MCU:** STM32G474RETx (Nucleo-G474RE)
- **Motor Driver:** Cytron MD20A (Sign-Magnitude PWM)
- **Power:** 24VDC, Mean Well PSP-750-24 (31.3A)
- **Joystick:** XInput controller ผ่าน Raspberry Pi Pico/ESP32 → USART3
- **UI:** Modbus RTU ผ่าน LPUART1

**สถานะปัจจุบัน:**
- Infrastructure (UART, DMA, Timer, State Machine, Modbus, Joystick) ✅ ทำงานได้
- Motor: หมุนได้ ผ่านการทดสอบ Mode 2 (Joystick) และ Mode 3 (Auto)
- Current Sensor: ทำงานได้ บน PC4 (ADC2_IN5)
- **งานที่เหลือ:** Homing logic, PID position control (STATE_AUTO)

---

## 2. Pin Mapping (ยืนยันจาก .ioc)

### Motor & Encoder
- **PA6** → TIM3_CH1 AF2 → Motor PWM (20kHz, ARR=8499)
- **PA0** → MOTOR_DIR → HIGH=Forward, LOW=Reverse
- **PA8** → TIM1_CH1 AF6 → Encoder B
- **PA9** → TIM1_CH2 AF6 → Encoder A
- **TIM1 Mode:** TIM_ENCODERMODE_TI12 (4x resolution)

### Sensors & Inputs
- **PC4** → ADC2_IN5 → WCS1800 Current Sensor (VCC=5V, zero=2.5V, sens=66mV/A)
- **PA1** → REED_UP (Pull-up)
- **PA4** → REED_DOWN (Pull-up)
- **PB0** → REED_GRIP (Pull-up)
- **PB13** → ESTOP (EXTI Falling, priority 0)
- **PB5** → MODE switch (EXTI Rising+Falling)
- **PB1** → RESET_BTN (Pull-up)
- **PB10** → POWER_BTN (Pull-up, 3s hold = shutdown)

### Outputs
- **PC1** → PNEUMATIC
- **PB11** → GRIPPER
- **PC7** → TOWER_G, **PC8** → TOWER_R, **PB7** → TOWER_Y
- **PB6** → EMER_OUTPUT (HIGH เมื่อ E-Stop active)
- **PB14** → POWER_LATCH (ต้อง SET HIGH เพื่อ latch ไฟ standalone)
- **PB4** → RESET_LED

### Communication
- **PA2/PA3** → LPUART1 TX/RX → Modbus RTU 19200 8E1 (DMA)
- **PC10/PC11** → USART3 TX/RX → Joystick XInput 460800 8N1 (DMA)

### ขาที่ยังไม่ได้ใช้ (สำคัญ!)
- **HOME Sensor (Proximity):** ยังไม่ได้ assign ขา → ต้องกำหนดก่อน implement Homing
- ขาว่างที่มี ADC: PA7(ADC2_IN4), PC2(ADC2_IN8), PC3(ADC2_IN9), PC5(ADC2_IN11)

---

## 3. Timing Architecture

```
170MHz Clock
├─ TIM2 → HAL Tick (SysTick replacement)
├─ TIM3 → PWM 20kHz (ARR=8499, PSC=0) บน PA6
├─ TIM1 → Encoder (ARR=65535, PSC=0)
└─ TIM7 → 1kHz ISR (ARR=999, PSC=169)
           ├─ อ่าน Encoder → current_position (delta accumulation)
           ├─ Poll POWER_BTN (3s hold logic)
           ├─ modbus_tick_1ms() (Software timeout)
           └─ sub_loop_counter++ → flag_10ms ทุก 10 รอบ

Main Loop while(1):
└─ if(flag_10ms) @ 100Hz
    ├─ ADC Read (Polling, 47.5 cycles sampling)
    ├─ Current EMA filter + WCS1800 conversion → current_sensor_A
    ├─ modbus_process()
    ├─ อัพเดท Modbus registers
    ├─ เช็ค MODE switch (edge detection)
    ├─ System Mode → State Machine
    ├─ Motor Apply (ramp + dead-time + SET_COMPARE)
    ├─ อัพเดท dev_dash (Live Expressions)
    └─ HAL_IWDG_Refresh()
```

---

## 4. Motor Control Details

### PWM Setup
```c
TIM3_CH1, PA6, AF2
Prescaler = 0, Period = 8499
→ 170MHz / 8500 = 20,000 Hz (20kHz, silent)
```

### Apply Pipeline
```c
// 1. State machine sets: motor_speed_cmd = -1.0 to +1.0
// 2. Cap by max_speed
// 3. Direction change dead-time (50ms = 5 cycles @ 100Hz)
// 4. Slew rate: ±ramp_rate per 10ms
// 5. Apply:
HAL_GPIO_WritePin(MOTOR_DIR, speed >= 0 ? SET : RESET);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
    (uint32_t)(fabsf(speed) * htim3.Init.Period));
```

### Known Issues & Solutions
| ปัญหา | สาเหตุ | แก้ |
|------|--------|-----|
| PSU ตัดเมื่อ speed สูง | Motor inrush > OCP threshold | ลด max_speed, เพิ่ม Cap 4700µF×3 |
| MD20A ERR | Under-voltage จาก PSU trip | เพิ่ม Capacitor bank |
| Back-EMF spike | Direction reversal ขณะ motor หมุน | Dead-time 50ms (implemented) |

---

## 5. Dashboard Structure (dev_dash)

```c
DevDashboard_t dev_dash = {
  .Ctrl = {
    .mode,             // 0=Production, 1=HW Test, 2=Joy Test, 3=Auto Test
    .force_*,          // Force outputs ใน mode 1
    .ramp_rate,        // 0.03f default (slew rate per 10ms)
    .max_speed,        // 0.40f default (hard cap)
    .auto_speed,       // 0.30f (mode 3 speed)
    .auto_period_fwd_ms, // 1000 ms
    .auto_period_rev_ms, // 1000 ms
    .cur_zero_v,       // 2.50f (WCS1800 @ 5V)
    .cur_sens,         // 0.066f (66mV/A)
  },
  .Joy = { connected, raw_buttons, L_Y, R_T, L_T },
  .Status = {
    .state,            // RobotState_t
    .motor_cmd,        // signed speed ที่ส่งออกจริง
    .encoder,          // current_position (int32)
    .current_A,        // Amperes float, 4 decimal places
  },
  .In = { estop, mode_switch, reset, power },
  .Out = { pwm, dir, pneumatic, gripper, tower_g/y/r, reset_led, emer }
}
```

---

## 6. Global Variables สำหรับ Control Team

```c
// ใน main.c — accessible จากทุก module ผ่าน extern
extern float    motor_speed_cmd;   // เขียน: สั่งความเร็ว (-1.0 ถึง +1.0)
extern int32_t  current_position;  // อ่าน: encoder position (อัพเดทที่ 1kHz)
extern float    current_sensor_A;  // อ่าน: motor current (A, อัพเดทที่ 100Hz)
```

---

## 7. State Machine สำหรับ Control Team

### STATE_CALIBRATE — ต้อง Implement
```c
// ต้องการก่อน: กำหนด HOME sensor ให้ขาหนึ่ง แล้วตั้งใน .ioc
case STATE_CALIBRATE:
    mb_slave.registers[0x27] = 1;
    motor_speed_cmd = -0.1f; // เคลื่อนไปหา Home (ทิศที่ถูกต้อง)
    // เมื่อ HOME sensor trigger:
    //   __HAL_TIM_SET_COUNTER(&htim1, 32768); // ตั้ง encoder ที่กึ่งกลาง range
    //   current_position = 0;
    //   current_state = STATE_IDLE;
    break;
```

### STATE_AUTO — PID Loop (Control Team ใส่ที่นี่)
```c
case STATE_AUTO:
    mb_slave.registers[0x27] = 8;
    int32_t target = (int32_t)(int16_t)mb_slave.registers[0x24];
    int32_t error = target - current_position;
    // implement PID:
    motor_speed_cmd = Kp*error + Ki*integral + Kd*derivative;
    // Clamp ให้อยู่ใน -1.0 ถึง +1.0
    break;
```

---

## 8. Files ที่สำคัญ

| File | ความสำคัญ |
|------|---------|
| `Core/Src/main.c` | Logic ทั้งหมด (State Machine, Motor, ADC, Dashboard) |
| `Core/Inc/main.h` | Pin defines ทั้งหมด |
| `Core/Src/joystick.c/h` | Parser และ API สำหรับ XInput |
| `Core/Src/modbus.c/h` | Modbus RTU slave implementation |
| `1-Dof-full-control.ioc` | CubeMX config (อ้างอิง pin mapping) |

---

## 9. สิ่งที่ต้องระวัง

1. **ADC Sampling Time** — ทุกครั้งที่ Generate Code ใหม่ จะ reset กลับเป็น `2CYCLES_5` → ต้องแก้เป็น `47CYCLES_5` ใน `MX_ADC2_Init()`

2. **TIM3 ใน .ioc** — ตั้งเป็น "PWM Generation1 No Output" แทน "CH1" → PA6 ยังได้ PWM อยู่เพราะ GPIO config ใน MX_GPIO_Init แต่ถ้า Generate Code ใหม่อาจมีปัญหา ควรแก้เป็น "PWM Generation CH1"

3. **POWER_LATCH (PB14)** — ถูก comment ออกในโค้ด ถ้าจะใช้ standalone (ไม่มี ST-Link) ต้อง uncomment: `HAL_GPIO_WritePin(POWER_LATCH_GPIO_Port, POWER_LATCH_Pin, GPIO_PIN_SET);`

4. **HOME Sensor** — ยังไม่ได้ assign ขา ต้องกำหนดก่อน implement Homing

5. **Capacitor Bank** — PSU ยังตัดที่ speed สูง ต้องใส่ 4700µF×3 / 50V ขนาน VIN ของ MD20A
