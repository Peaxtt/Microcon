# 🤖 Project Context: 1-DOF Robot Control System (Hardware-PCB Mapped)

**Microcontroller:** STM32G474RE (Nucleo-G474RE)
**Control Strategy:** Current/Torque Control using ADC + PI Controller

---

## 1. 📍 Hardware Pin Mapping (อิงตามสกรีนบนแผ่น PCB)

### 🔴 ฝั่งซ้าย (Header 8 ช่อง)
| ชื่อบน PCB (Label) | STM32 Pin | IO Type | หน้าที่การทำงาน (Function) |
| :--- | :--- | :--- | :--- |
| **`LM2`** | PB1 | Input | รับสัญญาณ Limit 1 (หรือปุ่ม Reset) |
| **`EMER`** | PB13 | EXTI (Priority 0) | รับสัญญาณปุ่ม E-Stop (จาก Opto) |
| **`PROX`** | PB5 | EXTI (Priority 0) | รับสัญญาณ Proximity / Home (จาก Opto) |
| **`IR`** | PB4 | Output | ส่งสัญญาณคุมไฟ LED ปุ่ม Reset (ไป Relay ช่อง 6) |
| **`LM1`** | PB10 | Input | รับสัญญาณปุ่ม Power (จาก Opto) |
| **`EN1`** | PB7 | Output | ส่งสัญญาณไฟ Tower Light สีเหลือง (ไป Relay ช่อง 2) |
| **`Servo`** | PC8 | Output | ส่งสัญญาณไฟ Tower Light สีแดง (ไป Relay ช่อง 3) |
| **`EN2`** | PC6 *(แนะนำ)*| Output | ✨ **EMER_OUTPUT:** ว่างอยู่! เอามาส่งสัญญาณไปตัด Relay ช่อง 8 (Software E-Stop) |

### 🔴 ฝั่งขวา (Header 10 ช่อง)
| ชื่อบน PCB (Label) | STM32 Pin | IO Type | หน้าที่การทำงาน (Function) |
| :--- | :--- | :--- | :--- |
| **`EN+`** | PA6 | PWM (TIM3) | ส่งสัญญาณ PWM ไป Cytron (สายสีขาว) |
| *(ใต้ EN+)* | PC5 | ADC1_IN11 | รับสัญญาณ Current Sensor |
| **`PRIS`** | PA1 | Output | ส่งสัญญาณ Direction ไป Cytron (สายสีเหลือง) |
| **`DEL`** | PC1 | Output | สั่ง Gripper Up/Down (ไป Relay ช่อง 4) |
| **`SAVE`** | PB11 | Output | สั่ง Gripper Close/Open (ไป Relay ช่อง 5) |
| **`HOME`** | PC0 | Input | รับสัญญาณปุ่ม Mode (จาก Opto) |
| **`DI`** | PC7 | Output | ส่งสัญญาณไฟ Tower Light สีเขียว (ไป Relay ช่อง 1) |
| **`O2 (บน)`** | PA9 | TIM1_CH2 | รับสัญญาณ Encoder A (สายเหลือง 8 คอร์) |
| **`O2 (ล่าง)`**| PA8 | TIM1_CH1 | รับสัญญาณ Encoder B (สายเขียว 8 คอร์) |
| **`RPT`** | PB14 | Output | **POWER_LATCH:** สั่งปิดเครื่อง (ไป Relay ช่อง 7) |

### 🔴 พินดำแยก (จิ้มตรงบนบอร์ด) & พอร์ตสื่อสาร
| ชื่อจุดเชื่อมต่อ | STM32 Pin | IO Type | หน้าที่การทำงาน (Function) |
| :--- | :--- | :--- | :--- |
| **พินดำ (REED_U)** | PA0 | Input | รับสัญญาณ Test Station: Reed Up |
| **พินดำ (REED_D)** | PA4 | Input | รับสัญญาณ Test Station: Reed Down |
| **พินดำ (REED_G)** | PB0 | Input | รับสัญญาณ Test Station: Reed Gripper |
| **USB (ST-Link)** | PA2, PA3 | LPUART1 | ส่งข้อมูล Modbus UI |
| **พินดำ (UART)** | PC10, PC11| USART3 | รับส่งข้อมูล Joystick (ย้ายหนี PB10/PB11) |

---

## 2. ⚡ NVIC Priority & Architecture (4-bit Preemption)
* **Priority 0 (Safety First):** `EXTI15:10` (PB13 EMER), `EXTI9:5` (PB5 PROX), `ADC1` (Current Sensor).
* **Priority 1 (Feedback):** `TIM1` (Encoder PA8, PA9).
* **Priority 2 (Control Loop):** `TIM7` (1kHz loop for PI Current Control).
* **Priority 3 (Communication):** `LPUART1`, `USART3`.
* **Priority 15 (System):** `TIM2` (HAL Timebase).

---

## 3. 🛠️ Strict Instructions for AI Assistant:
1. Always use the STM32 HAL Library and strictly follow the Pin mapping matched with the PCB labels above.
2. Ensure the PI Current Controller runs inside the `TIM7` (1kHz) interrupt.
3. If `PB13` (EMER) is triggered or ADC detects over-current, immediately halt PWM and enter `STATE_ERROR`.
4. If the Joystick Software E-Stop is pressed, pull `PC6` (`EN2` / EMER_OUTPUT) HIGH to cut the hardware relay on Channel 8.