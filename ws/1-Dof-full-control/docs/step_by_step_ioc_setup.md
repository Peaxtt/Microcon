# Step-by-Step STM32CubeMX (`.ioc`) Setup Guide

This guide provides the exact click-by-click instructions to configure the `1-Dof-full-control.ioc` file in STM32CubeMX or STM32CubeIDE to match our "Perfect" Industrial-Grade Architecture.

Please open your `.ioc` file and follow these steps in the **Pinout & Configuration** tab.

---

## 1️⃣ System Core

1.  **RCC:**
    *   `High Speed Clock (HSE)`: Select **Bypass Clock Source** (if using ST-Link clock) or **Crystal/Ceramic Resonator**.
2.  **SYS:**
    *   `Debug`: Select **Serial Wire**.
    *   `Timebase Source`: Change from SysTick to **TIM2** (Ensures HAL delays work safely even inside RTOS/Interrupts).
3.  **IWDG:**
    *   Check `Activated`.
    *   *Parameter Settings:* Prescaler = **32**, Window value = **4095**, Reload value = **4095**. (This creates a ~3-4 second hardware watchdog timeout).
4.  **NVIC:** (Click NVIC, configure in the right pane later, see Step 6).
    *   *Priority Group:* Select **4 bits for pre-emption priority**.

*(Optional but Recommended)* To fix the LPUART1 19200 Baudrate limit:
*   Go to the **Clock Configuration** tab.
*   Locate the **LPUART1 Clock Mux** (center-right).
*   Change the source from `PCLK` to **`HSI`**. This allows LPUART1 to support lower baud rates like 19200.

---

## 2️⃣ Analog

1.  **ADC1:**
    *   Check **IN11 Single-ended** (Pin `PC5` will turn green).
    *   *Parameter Settings:*
        *   `Continuous Conversion Mode`: **Enable**.
    *   *(No Analog Watchdog needed, using hardware fuse).*
    *   *NVIC Settings:* (Optional) You can disable ADC interrupts if you plan to read it by polling in the 1kHz loop.

---

## 3️⃣ Timers

1.  **TIM1 (Encoder):**
    *   `Combined Channels`: Select **Encoder Mode** (Pins `PA8`, `PA9` will turn green).
    *   *Parameter Settings:* Expand `Input Capture Channel 1/2` and set `IC1 Filter` and `IC2 Filter` to **4** or **8** (Hardware debouncing).
2.  **TIM3 (Motor PWM):**
    *   `Channel 1`: Select **PWM Generation CH1** (Pin `PA6`).
    *   *Parameter Settings:*
        *   `Prescaler`: **0**
        *   `Counter Period`: **8499** (Yields ~20kHz PWM on a 170MHz clock for a Silent Motor).
3.  **TIM7 (Control Loop 1kHz):**
    *   Check **Activated**.
    *   *Parameter Settings:* `Prescaler` = **169**, `Counter Period` = **999** (Yields 1000Hz interrupt).
    *   *NVIC Settings:* Check **TIM7 global interrupt**.

---

## 4️⃣ Connectivity

1.  **LPUART1 (Modbus UI):**
    *   `Mode`: **Asynchronous**.
    *   *Parameter Settings:*
        *   `Baud Rate`: **19200**
        *   `Word Length`: **8 Bits** (includes parity).
        *   `Parity`: **Even**.
        *   `Stop Bits`: **1**.
    *   *Advanced Features:* `Receiver Timeout Enable` = **Enable** (Set Length to **4** bits). *Note: If you cannot find this option in your CubeMX version, skip it! We will handle the timeout in software using our 1kHz TIM7 loop.*
    *   *DMA Settings:* Click `Add`.
        *   Add **LPUART1_RX** (Mode: Normal, Data Width: Byte).
        *   Add **LPUART1_TX** (Mode: Normal, Data Width: Byte).
    *   *NVIC Settings:* Check **LPUART1 global interrupt**.
2.  **USART3 (Joystick):**
    *   `Mode`: **Asynchronous**.
    *   *Parameter Settings:* `Baud Rate`: **460800** (Match your Usb-Reader), `Parity`: **None**.
    *   *DMA Settings:* Click `Add` -> Add **USART3_RX** (Mode: Normal, Data Width: Byte).
    *   *NVIC Settings:* Check **USART3 global interrupt**.

---

## 5️⃣ GPIO (Digital I/O & EXTI)

Click the **GPIO** tab to see the list of pins. Left-click the pins on the chip graphic to set their mode, then right-click to enter the `User Label`.

*   **Outputs (`GPIO_Output`):**
    *   `PA1` -> Label: **MOTOR_DIR**
    *   `PC7` -> Label: **TOWER_G**
    *   `PB7` -> Label: **TOWER_Y**
    *   `PC8` -> Label: **TOWER_R**
    *   `PB6` -> Label: **EMER_OUT** (Emergency Output Signal)
    *   `PC1` -> Label: **PNEUMATIC**
    *   `PB11` -> Label: **GRIPPER**
    *   `PB4` -> Label: **RESET_LED**
    *   `PB14` -> Label: **POWER_LATCH**
*   **Inputs (`GPIO_Input` - Enable Pull-up in GPIO config panel):**
    *   `PB1` -> Label: **RESET_BTN**
    *   `PB10` -> Label: **POWER_BTN**
    *   `PC0` -> Label: **MODE_BTN**
    *   `PA0` -> Label: **REED_UP**
    *   `PA4` -> Label: **REED_DOWN**
    *   `PB0` -> Label: **REED_GRIP**
*   **EXTI (Hardware Interrupts - Enable Pull-up):**
    *   `PB13` -> Select `GPIO_EXTI13`, Label: **ESTOP**. *GPIO Mode:* **External Interrupt Mode with Falling edge trigger detection**.
    *   `PB5` -> Select `GPIO_EXTI5`, Label: **HOME**. *GPIO Mode:* **External Interrupt Mode with Rising/Falling edge trigger detection**.

---

## 6️⃣ Final NVIC Priority Configuration

Go back to **System Core -> NVIC** and set the `Preemption Priority` column exactly as follows (0 is highest priority):

1.  **EXTI line[15:10] interrupts:** **0** (E-Stop is critical safety).
2.  **EXTI line[9:5] interrupts:** **0** (Home Sensor).
3.  **ADC1 and ADC2 global interrupt:** **0** (Analog Watchdog safety).
4.  **TIM1 global / break:** **1** (Encoder).
5.  **TIM7 global interrupt:** **2** (1kHz Control Loop).
6.  **LPUART1 global interrupt:** **3** (Modbus RTO).
7.  **USART3 global interrupt:** **3** (Joystick RX).
8.  **All DMA channel interrupts:** **3**.

---

### 🎉 Next Steps
Once you have completed these steps in STM32CubeMX:
1. Click **Save (`Ctrl+S`)** or click the **Generate Code** gear icon.
2. Let me know when the code is generated, and I will write the ultimate State Machine and Modbus C code!