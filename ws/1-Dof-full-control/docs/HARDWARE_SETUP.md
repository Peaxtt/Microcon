# 🔧 Hardware Setup

## 📍 Pinout Mapping
*อัปเดตข้อมูลล่าสุดจากไฟล์ .ioc*

### Motor & Encoder
* **PA0:** Motor Direction (`MOTOR_DIR`)
* **PC6:** Motor PWM (`MOTOR_PWM` - TIM3_CH1)
* **PA9:** Encoder Phase A (TIM1_CH2)
* **PA8:** Encoder Phase B (TIM1_CH1)

### Digital Inputs
* **PB13:** Emergency Stop (E-Stop)
* **PB5:** Auto/Manual Selector Switch (`MODE`)
* **PB1:** Reset Button
* **PB10:** Power Button
* **PA1:** Reed Switch: Up
* **PA4:** Reed Switch: Down
* **PB0:** Reed Switch: Gripper

### Digital Outputs (Relays)
* **PC7:** Tower Light: Green
* **PB7:** Tower Light: Yellow
* **PC8:** Tower Light: Red
* **PC1:** Pneumatic Cylinder: Up/Down
* **PB11:** Pneumatic Gripper: Close/Open
* **PB4:** Reset LED Indicator
* **PB14:** System Power Latch
* **PB6:** Emergency Output Signal (EMER_OUT)

### Analog
* **PC5:** Current Sensor (ADC2_IN11)
