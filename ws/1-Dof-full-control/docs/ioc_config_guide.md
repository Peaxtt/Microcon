# STM32CubeMX (`.ioc`) Configuration Guide

Based on the required Modbus settings and hardware pin mapping, please update the `.ioc` file in STM32CubeMX / STM32CubeIDE with the following configurations.

## 1. System Core
* **SYS:** Debug = Serial Wire
* **RCC:** High Speed Clock (HSE) = Bypass Clock Source / Crystal (depending on your board, usually Nucleo uses Bypass from ST-Link).

## 2. Analog
* **ADC1:**
  * **IN11 (PC5):** Select `IN11 Single-ended`.
  * This is used for the **Current Sensor**.

## 3. Timers
* **TIM1 (Encoder):**
  * **Combined Channels:** Encoder Mode.
  * Ensures `PA8` (CH1) and `PA9` (CH2) are mapped correctly.
* **TIM3 (Motor PWM):**
  * **Channel 1:** PWM Generation CH1.
  * Ensures `PA6` is mapped as TIM3_CH1.
  * **Configuration:** Set Prescaler and Counter Period to achieve the desired PWM frequency (e.g., 10kHz to 20kHz for Cytron MD20A).

## 4. Connectivity (UART / Modbus / Joystick)
* **LPUART1 (Modbus UI Interface):**
  * Mode: **Asynchronous**
  * Hardware Flow Control: Disable
  * **Parameter Settings:**
    * Baud Rate: **19200**
    * Word Length: **8 Bits**
    * Parity: **Even**
    * Stop Bits: **1**
  * Pins: `PA2` (TX) and `PA3` (RX).
  * **NVIC Settings:** Enable LPUART1 global interrupt.

* **USART3 (Joystick Interface via DMA):**
  * Mode: **Asynchronous**
  * Hardware Flow Control: Disable
  * **Parameter Settings:**
    * Baud Rate: **460800** (or whatever the Usb-Reader uses)
    * Word Length: 8 Bits
    * Parity: None
    * Stop Bits: 1
  * Pins: `PC10` (TX) and `PC11` (RX). *(Note: Ensure these pins match your physical connection, the generated code used PC10/11 based on typical USART3 alternate functions, but please verify with your board).*
  * **DMA Settings:**
    * Add DMA Request for **USART3_RX**.
    * Mode: Normal.
    * Data Width: Byte.
  * **NVIC Settings:** Enable USART3 global interrupt and the corresponding DMA Channel interrupt.

## 5. GPIO Mapping (Digital I/O)

### Motor Control
* `PA1`: GPIO_Output (Label: `MOTOR_DIR`)

### Sensors & Buttons (Inputs - Pull-up recommended if not handled externally)
* `PB13`: GPIO_Input (Label: `ESTOP`)
* `PB5`: GPIO_Input (Label: `HOME`)
* `PB1`: GPIO_Input (Label: `RESET_BTN`)
* `PB10`: GPIO_Input (Label: `POWER_BTN`)
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
* `PB14`: GPIO_Output (Label: `POWER_LATCH`)

*(Ensure all outputs start with the correct default level, usually Low/Reset, depending on your relay module's active state).*

## 6. Interrupts (NVIC)
Ensure the priorities are set correctly. Usually:
1. E-Stop / EXTI (Highest)
2. DMA (Joystick RX)
3. UART (Modbus / Joystick)
4. Systick / Timers
