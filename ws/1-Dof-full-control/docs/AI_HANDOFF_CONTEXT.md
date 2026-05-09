# 🤖 1-DOF Robot System - AI Developer Context Handoff

This document contains the complete and finalized context of the 1-DOF Robot Control System. It is designed to be fed to an AI assistant (like Claude or ChatGPT) to instantly understand the system architecture, hardware mappings, state machine, and communication protocols before continuing development (e.g., adding PID control).

---

## 🎯 1. Project Overview & Architecture
*   **Microcontroller:** STM32G474RE (Nucleo-G474RE)
*   **Core Philosophy:** "Zero CPU Overhead" Industrial-Grade Architecture.
*   **Timebase Control:**
    *   `SysTick` is separated to `TIM2` for safe HAL delays.
    *   `TIM7` runs a strict **1kHz (1ms) Interrupt Loop**. This is where PID control, sensor polling, and safety checks happen.
    *   Inside the 1kHz loop, a `100Hz` flag (`flag_10ms`) is triggered to run the State Machine and Modbus UI updates in the main `while(1)` loop, preventing heavy UI tasks from choking the motor control.
*   **Safety Features:**
    *   **IWDG:** Independent Watchdog enabled (resets board if frozen for >3 seconds).
    *   **EXTI:** E-Stop (`PB13`) and Selector Switch (`PB5`) are wired to Hardware Interrupts for microsecond response times.
*   **Communication:**
    *   **UI (Modbus RTU):** `LPUART1` (19200 8E1) running on DMA with Software Timeout inside the 1kHz loop.
    *   **Joystick (XInput):** `USART3` (460800 8N1) receiving 15-byte packets via DMA from a Raspberry Pi Pico.

---

## 📍 2. Pinout & Hardware Mappings
*Verified against .ioc file*

### Motor & Sensors
*   **Motor PWM:** `PC6` (TIM3_CH1) - Configured for 20kHz (Silent Motor).
*   **Motor DIR:** `PA0` (GPIO Output).
*   **Encoder:** `PA8` & `PA9` (TIM1 in Encoder Mode).
*   **Current Sensor:** `PC5` (ADC2_IN11) - Read every 10ms with an Exponential Moving Average Filter.

### Digital Inputs (Pull-up configured)
*   `PB13` -> E-Stop Button (EXTI13 - Falling Edge)
*   `PB5`  -> Auto/Manual Selector Switch (EXTI5)
*   `PB1`  -> Reset Button
*   `PB10` -> Power Button (Polled at 1kHz. 3-second hold triggers shutdown).
*   `PA1`  -> Reed Switch: Up
*   `PA4`  -> Reed Switch: Down
*   `PB0`  -> Reed Switch: Gripper

### Digital Outputs & Relays
*   `PC7`  -> Tower Light: Green
*   `PB7`  -> Tower Light: Yellow
*   `PC8`  -> Tower Light: Red
*   `PB6`  -> Emergency Output Signal (`EMER_OUTPUT_Pin`) - Outputs 3.3V when in EMER state.
*   `PC1`  -> Pneumatic Cylinder (Up/Down)
*   `PB11` -> Gripper (Close/Open)
*   `PB4`  -> Reset LED Indicator
*   `PB14` -> Power Latch (Driven LOW to shut down main cabinet power).

---

## 🚦 3. State Machine Logic

The robot operates strictly within a 5-State Machine defined by the `RobotState_t` enum:

1.  **`STATE_IDLE` (Modbus 0x27 = 0):** Motor holds position. Waits for user or Modbus commands to change state. Automatically transitions to `STATE_AUTO` or `STATE_MANUAL` based on the physical selector switch (`PB5`).
2.  **`STATE_CALIBRATE` (Modbus 0x27 = 1):** Moves to find the Home position.
3.  **`STATE_MANUAL` (Modbus 0x27 = 0):** Controlled purely by Joystick analog inputs or Modbus Jog commands. Bypasses P2P logic.
4.  **`STATE_AUTO` (Modbus 0x27 = 8):** Executes Point-to-Point (P2P) or Pick & Place sequences. **(This is where the future PID algorithms should take control of `motor_speed_cmd`).**
5.  **`STATE_EMER` (Modbus 0x27 = 0):** Triggered by `PB13` (Hard E-Stop) or `L3+R3` on Joystick. Instantly forces PWM to 0, sets `EMER_OUTPUT` high, and turns on Tower Red. Exiting requires releasing the E-stop and pressing the Back button (Reset).

---

## 🎛️ 4. Testing Modes & Live Expressions

A `DevDashboard_t dev_dash` struct is available globally to monitor the entire system in real-time via STM32CubeIDE Live Expressions.

The system supports a `system_mode` variable inside the dashboard:
*   **`0` (`SYS_MODE_PRODUCTION`):** Normal state machine execution.
*   **`1` (`SYS_MODE_HARDWARE_TEST`):** Bypasses all logic. Allows developers to manually change boolean variables in `dev_dash` to force relays ON/OFF and command motor speeds directly from the PC.
*   **`2` (`SYS_MODE_JOYSTICK_TEST`):** Bypasses all logic. Maps Joystick buttons directly to output pins.
*   **`3` (`SYS_MODE_AUTO_MOTOR_TEST`):** Foolproof hardware validation. Toggles motor direction every 1 second at 30% speed.

---

## 📡 5. Modbus Register Map (Slave ID: 21)

**WRITE (PC -> STM32):**
*   `0x00`: Heartbeat (Expects 18537)
*   `0x01`: Mode Command (1=Go Home, 2=Manual, 4=Auto, 8=Set Home, 16=Test)
*   `0x24`: P2P Target Position (Signed int16)

**READ (STM32 -> PC):**
*   `0x00`: Heartbeat Response (Sends 22881)
*   `0x26`: Reed Sensors State
*   `0x27`: Current Task (0=Idle, 1=Homing, 8=P2P)
*   `0x28`: Encoder Position
*   `0x31`: Emergency State (1=Active)

---
**DEVELOPER NOTE FOR THE NEXT AI:**
The core infrastructure (UART, DMA, Timers, State Machine, Modbus parsing, Joystick parsing) is 100% complete and verified working. Your task is to implement the physical control algorithms (e.g., PID loop inside the 1kHz `TIM7` callback) and populate the Modbus sequence logic within the `STATE_AUTO` block. Good luck!
