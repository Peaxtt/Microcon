# STM32CubeMX (`.ioc`) Configuration Guide

Based on the required Modbus settings, hardware pin mapping, and the optimized Industrial-Grade architecture, please update the `.ioc` file in STM32CubeMX / STM32CubeIDE with the following configurations.

## 1. System Core
* **SYS:** Debug = Serial Wire
* **RCC:** High Speed Clock (HSE) = Bypass Clock Source / Crystal.
* **IWDG (Independent Watchdog):** Enable. Set prescaler and reload value for a ~1-2 second timeout.
* **NVIC (Priority Management):** Set Priority Group to 4 bits for preemption priority.
  * **Priority 0 (Highest):** EXTI line[15:10] (E-Stop) & ADC1_2 global interrupt (Analog Watchdog / Break).
  * **Priority 1:** TIM1 break, update, trigger and commutation interrupts / TIM1 global (Encoder).
  * **Priority 2:** TIM7 global interrupt (1kHz Control Loop).
  * **Priority 3:** DMA1 channel1 global interrupt (Modbus/Joystick RX) & LPUART1/USART3 global interrupts.

## 2. Analog
* **ADC1:**
  * **IN11 (PC5):** Select `IN11 Single-ended` (Current Sensor).
  * **Analog Watchdog 1:** Enable. Set high/low thresholds for overcurrent detection (Stall detection) to trigger an interrupt (Priority 0) and route to TIM3 Break Input.

## 3. Timers
* **TIM1 (Encoder):**
  * **Combined Channels:** Encoder Mode.
  * Ensures `PA8` (CH1) and `PA9` (CH2) are mapped correctly.
  * **Configuration:** Enable Digital Filters on TI1 and TI2 (e.g., set filter to 4 or 8) to debounce noise.
* **TIM3 (Motor PWM):**
  * **Channel 1:** PWM Generation CH1.
  * Ensures `PA6` is mapped as TIM3_CH1.
  * **Configuration:** Set Prescaler and Counter Period to achieve a PWM frequency of **≥ 20 kHz** (e.g., for 170MHz clock, Prescaler=0, Period=8499 gives ~20kHz). This ensures a "Silent Motor".
  * **Break-Feature:** Enable BKIN (Break Input) and route it to the ADC1 Analog Watchdog internally.
* **TIM7 (Control Loop):**
  * **Configuration:** Set to generate an interrupt exactly at **1000Hz (1ms)**.
  * Used for PID, polling Power Button, and running the 100Hz UI/State sub-loop.

## 4. Connectivity (UART / Modbus / Joystick)
* **LPUART1 (Modbus UI Interface):**
  * Mode: **Asynchronous**
  * Hardware Flow Control: Disable
  * **Parameter Settings:**
    * Baud Rate: **19200**
    * Word Length: **8 Bits**
    * Parity: **Even**
    * Stop Bits: **1**
  * **Hardware Features:** Enable **Receiver Timeout (RTO)** and set it to 3.5 character times (for Modbus RTU end-of-frame detection).
  * **DMA Settings:** Enable DMA for RX and TX to reduce CPU load.
  * **NVIC Settings:** Enable LPUART1 global interrupt (Priority 3).

* **USART3 (Joystick Interface via DMA):**
  * Mode: **Asynchronous**
  * Hardware Flow Control: Disable
  * **Parameter Settings:**
    * Baud Rate: **460800**
    * Word Length: 8 Bits
    * Parity: None
    * Stop Bits: 1
  * Pins: `PC10` (TX) and `PC11` (RX).
  * **DMA Settings:** Add DMA Request for **USART3_RX**. Mode: Normal.
  * **NVIC Settings:** Enable USART3 global interrupt and DMA Channel interrupt (Priority 3).

## 5. GPIO Mapping (Digital I/O)

### Motor Control
* `PA1`: GPIO_Output (Label: `MOTOR_DIR`)

### Sensors & Buttons (Inputs)
* `PB13`: **EXTI Line 13** (Label: `ESTOP`) -> Triggers interrupt on falling edge (Priority 0).
* `PB5`: **EXTI Line 5** (Label: `HOME`) -> Triggers interrupt on edge.
* `PB1`: GPIO_Input (Label: `RESET_BTN`)
* `PB10`: GPIO_Input (Label: `POWER_BTN`) -> Polled at 1kHz to check for 3-second hold.
* `PC0`: GPIO_Input (Label: `MODE_BTN`)
* `PA0`: GPIO_Input (Label: `REED_UP`)
* `PA4`: GPIO_Input (Label: `REED_DOWN`)
* `PB0`: GPIO_Input (Label: `REED_GRIP`)

### Relay Control (Outputs)
* `PC7`: GPIO_Output (Label: `TOWER_G`)
* `PB7`: GPIO_Output (Label: `TOWER_Y`)
* `PC8`: GPIO_Output (Label: `TOWER_R`)
* `PC1`: GPIO_Output (Label: `PNEUMATIC`)
* `PB11`: GPIO_Output (Label: `GRIPPER`)
* `PB4`: GPIO_Output (Label: `RESET_LED`)
* `PB14`: GPIO_Output (Label: `POWER_LATCH`) -> Drops LOW to shut down power.

*(Ensure all outputs start with the correct default level, usually Low/Reset).*

## 6. Storage
* Configure the project to allow erasing/writing to a specific Flash page (e.g., the last page of the STM32G474 memory) to securely store PID values (`Kp`, `Ki`, `Kd`) and Home Offset.