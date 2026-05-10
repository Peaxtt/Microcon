# Hardware Setup — 1-DOF Robot Control System

อ้างอิงจาก `.ioc` ล่าสุด (STM32G474RETx, Nucleo-G474RE)

---

## Pinout Mapping (ครบถ้วน)

### Motor & Encoder
| Pin | Label | Type | หมายเหตุ |
|-----|-------|------|---------|
| PA6 | MOTOR_PWM | TIM3_CH1 AF2 | PWM 20kHz, ความเร็วมอเตอร์ (Cytron MD20A) |
| PA0 | MOTOR_DIR | GPIO Output | ทิศทาง: HIGH=Forward, LOW=Reverse |
| PA8 | ENCODER_B | TIM1_CH1 AF6 | Encoder Phase B |
| PA9 | ENCODER_A | TIM1_CH2 AF6 | Encoder Phase A |

**TIM1 Encoder Mode: TI12 (4x resolution)** — นับทั้ง rising+falling ทั้ง 2 channel

### Analog Input
| Pin | Label | Type | หมายเหตุ |
|-----|-------|------|---------|
| PC4 | CURRENT_SENSOR | ADC2_IN5 | WCS1800 module, VCC=5V, zero=2.5V, sens=66mV/A |

### Digital Inputs
| Pin | Label | Pull | Mode | หมายเหตุ |
|-----|-------|------|------|---------|
| PB13 | ESTOP | None | EXTI Falling | Emergency Stop — ตอบสนองทันทีผ่าน Interrupt |
| PB5 | MODE | None | EXTI Rising+Falling | Auto/Manual selector switch |
| PB1 | RESET_BTN | Pull-up | GPIO Input | Reset alarm |
| PB10 | POWER_BTN | Pull-up | GPIO Input | กดค้าง 3 วินาที → shutdown |
| PA1 | REED_UP | Pull-up | GPIO Input | Reed switch ตำแหน่งบนสุด |
| PA4 | REED_DOWN | Pull-up | GPIO Input | Reed switch ตำแหน่งล่างสุด |
| PB0 | REED_GRIP | Pull-up | GPIO Input | Reed switch gripper |

### Digital Outputs
| Pin | Label | Type | หมายเหตุ |
|-----|-------|------|---------|
| PC1 | PNEUMATIC | GPIO Output | กระบอกลม Up/Down |
| PB11 | GRIPPER | GPIO Output | Gripper Close/Open |
| PB4 | RESET_LED | GPIO Output | LED แสดงสถานะ Reset |
| PC7 | TOWER_G | GPIO Output | Tower Light: Green |
| PC8 | TOWER_R | GPIO Output | Tower Light: Red |
| PB7 | TOWER_Y | GPIO Output | Tower Light: Yellow |
| PB6 | EMER_OUTPUT | GPIO Output | สัญญาณ output ฉุกเฉิน (HIGH เมื่อ EMER) |
| PB14 | POWER_LATCH | GPIO Output | ล็อคไฟระบบ (ต้อง SET ตอน startup สำหรับ standalone) |
| PA5 | LD2 | GPIO Output | Nucleo onboard LED (Heartbeat blink) |

### Communication
| Pin | Label | Peripheral | Settings |
|-----|-------|------------|---------|
| PA2 | LPUART1_TX | LPUART1 | Modbus RTU, 19200 8E1, DMA |
| PA3 | LPUART1_RX | LPUART1 | Modbus RTU, 19200 8E1, DMA |
| PC10 | USART3_TX | USART3 | Joystick XInput, 460800 8N1, DMA |
| PC11 | USART3_RX | USART3 | Joystick XInput, 460800 8N1, DMA |

---

## Timer Configuration

| Timer | Mode | Settings | ใช้สำหรับ |
|-------|------|----------|---------|
| TIM1 | Encoder TI12 | ARR=65535, PSC=0 | Quadrature Encoder (4x) |
| TIM2 | Time Base | — | HAL SysTick (ไม่ใช้ SysTick ตัวจริง) |
| TIM3 | PWM CH1 | ARR=8499, PSC=0 → 20kHz | Motor PWM output บน PA6 |
| TIM7 | Base IT | ARR=999, PSC=169 → 1kHz | 1kHz Control Loop |

---

## Power System

- **Bus Voltage:** 24VDC
- **PSU:** Mean Well PSP-750-24 (31.3A, OCP ~33A)
- **Motor Driver:** Cytron MD20A (20A dual channel, Sign-Magnitude PWM)
- **⚠️ แนะนำ:** ใส่ Capacitor 4700µF×3 / 50V ขนาน VIN ของ MD20A เพื่อรับ back-EMF spike

---

## Current Sensor (WCS1800)

- **Pin:** PC4 (ADC2_IN5, ADC2 Channel 5)
- **VCC:** 5V
- **Output ที่ 0A:** 2.5V (VCC/2)
- **Sensitivity:** 66mV/A
- **Sampling Time:** 47.5 cycles (กรองสัญญาณรบกวน)
- **ค่า Calibration ปัจจุบัน:** `cur_zero_v=2.5`, `cur_sens=0.066`

---

## ขาที่ยังไม่ได้ใช้ (ว่างอยู่)

ขาว่างที่มี ADC capability (สำหรับ HOME sensor หรือ sensor เพิ่มเติม):
- `PA7` → ADC2_IN4
- `PC2` → ADC2_IN8
- `PC3` → ADC2_IN9
- `PC5` → ADC2_IN11 (เคยใช้เป็น current sensor, ปัจจุบันว่าง)

**HOME Sensor (Proximity):** ยังไม่ได้ assign — รอการกำหนดขา
