# Ultimate System Architecture & Function Design

This document outlines the final "Perfect Setup" for the 1-DOF Robot Control System, leveraging the absolute most advanced hardware features of the STM32G474. This ensures extreme stability, zero-latency safety, and industrial-grade reliability for the Modbus UI, Joystick, and underlying control loops. 

---

## 🌟 1. Advanced Hardware Setup (Industrial Grade)

To achieve true perfection, we offload as much work as possible from the CPU to the STM32 hardware peripherals.

### A. Modbus RTU (LPUART1) - Zero CPU Overhead
*   **DMA RX & TX:** Use DMA for receiving and transmitting Modbus frames. This prevents CPU stalling when the UI sends large read requests.
*   **Hardware Receiver Timeout (RTO):** Instead of using a software timer or a separate hardware timer (like TIM6) to calculate Modbus T1.5/T3.5 frame endings, we utilize the STM32G4's built-in **UART Receiver Timeout (RTO)** feature. The UART hardware will automatically trigger an interrupt exactly when the bus is idle for the required character times. This is the absolute best way to handle Modbus RTU.

### B. Control Loop & State Machine Timing (TIM7)
*   **Constant Frequency Interrupt (1kHz):** **TIM7** generates a hardware interrupt exactly at 1000Hz (1ms). 
*   **Split Architecture:** 
    *   *1kHz Loop:* Calculates PID, polls the Power Button (`PB10`) for the 3-second hold, and reads the Encoder. This ensures smooth motor control.
    *   *100Hz Sub-Loop:* Uses a software counter inside the 1kHz loop to trigger Modbus register updates, Joystick parsing, and state machine transitions every 10ms. This prevents the UI from choking the motor control.

### C. Hardware-Level Safety & Protection
*   **EXTI (External Interrupts):** The E-Stop (`PB13`) and Home Sensor (`PB5`) use EXTI. Priority 0.
*   **Hardware PWM Break (BKIN):** We route the ADC Analog Watchdog (AWD) directly to the **TIM3 Break Input (BKIN)** internally. If the motor stalls and current spikes, the PWM pin is forced to its safe state (LOW) instantly by the hardware logic itself—requiring **0 CPU cycles**.
*   **Independent Watchdog (IWDG):** Crucial for industrial systems. If the software freezes or the main loop crashes, the IWDG will automatically reset the microcontroller. This forces the power latch pin (`PB14`) to drop, shutting down the entire robot safely.
*   **Hardware Encoder Filtering:** Digital filters are enabled on TIM1 (Encoder) to debounce the signals and reject electrical noise from the motor cables.

### D. Flash EEPROM Emulation
*   **Persistent Memory:** When the robot completes Homing or the PID values (`Kp`, `Ki`, `Kd`) are tuned via Modbus, the values are saved directly to the STM32 Flash memory. Upon reboot, the system loads these settings instantly.

---

## 🎮 2. Joystick & UI Integration (Shared State)

*   **Shared Register Array:** The `mb_regs[50]` array acts as the single source of truth.
*   **Analog Deadband:** Applied locally in software to the Joystick axes (LY/RX) to prevent motor whining when the stick doesn't return perfectly to 0.
*   When the Joystick is used, the code locally updates the exact same variables that the Modbus UI would update, ensuring the system state remains strictly synchronized.