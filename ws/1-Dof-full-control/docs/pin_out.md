# Pin Reference — 1-DOF Robot Control (STM32G474RETx)

**MCU:** STM32G474RET6 (LQFP64)  
**Board:** NUCLEO-G474RE (dev) → custom PCB (prod)  
**Last updated:** 2026-05-21

---

## Current Pin Assignments (from IOC)

### Power & System
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PA5 | LD2 | Output | Push-Pull | Onboard debug LED |
| PC13 | RESET_BTN | Input | PULLUP | Reset button (active LOW) |
| PC1 | POWER_LATCH | Output | Push-Pull | Power latch / cylinder DOWN actuator |
| PB10 | POWER_BTN | Input | PULLUP | Power button (active LOW) |
| PB4 | RESET_LED | Output | Push-Pull | Reset LED indicator |

### Motor Control
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PA6 | (TIM3_CH1) | PWM | TIM3 CH1 | Motor PWM 20kHz (period=8499) |
| PA0 | MOTOR_DIR | Output | Push-Pull | Motor direction (Cytron DIR) |
| PA8 | (TIM1_CH1) | Encoder | TIM1 Enc | Encoder channel A |
| PA9 | (TIM1_CH2) | Encoder | TIM1 Enc | Encoder channel B |

### Safety
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PC2 | ESTOP | Input | NOPULL | Emergency stop input — active LOW, debounced 30ms |
| PB14 | EMER_OUTPUT | Output | Push-Pull | Software E-Stop output relay |

### Actuators (outputs)
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PB11 | GRIPPER | Output | Push-Pull | Gripper close/open solenoid |
| PC6 | PNEUMATIC | Output | Push-Pull | Pneumatic solenoid (aux) |

### Tower Lights
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PC7 | TOWER_G | Output | Push-Pull | Tower light — Green |
| PB7 | TOWER_Y | Output | Push-Pull | Tower light — Yellow |
| PC8 | TOWER_R | Output | Push-Pull | Tower light — Red |

### Reed Switches (digital inputs)
| Pin | Label | Direction | Config | Function |
|-----|-------|-----------|--------|----------|
| PA1 | REED_UP | Input | **PULLDOWN** | Cylinder UP end-stop — active LOW |
| PA4 | REED_DOWN | Input | **PULLDOWN** | Cylinder DOWN end-stop — active LOW |
| PB0 | REED_GRIP | Input | **PULLDOWN** | Gripper CLOSED sensor — active LOW |

> **Wiring:** NC contact. COM → board 3V3. NC pin → GPIO.  
> Normal (no magnet): NC closed → 3V3 on GPIO → HIGH.  
> Triggered (magnet): NC opens → PULLDOWN → LOW.  
> `reed_up/down/grip = 1` means position reached.

> **IOC TODO:** Change PA1, PA4, PB0 from `GPIO_PULLUP` → `GPIO_PULLDOWN`

### Communication
| Pin | Label | Direction | Peripheral | Function |
|-----|-------|-----------|------------|----------|
| PA2 | LPUART1_TX | TX | LPUART1 | Modbus + Telemetry → PC (115200 8N1) |
| PA3 | LPUART1_RX | RX | LPUART1 | Modbus RX ← PC |
| PC10 | USART3_TX | TX | USART3 | Joystick bridge TX (460800 8N1) |
| PC11 | USART3_RX | RX | USART3 | Joystick bridge RX ← RP2040/XInput |

### Analog
| Pin | Label | Direction | Peripheral | Function |
|-----|-------|-----------|------------|----------|
| PC4 | (ADC2_IN5) | Analog In | ADC2 | Current sensor |

### Debug (SWD)
| Pin | Label | Peripheral |
|-----|-------|------------|
| PA13 | T_SWDIO | SWD |
| PA14 | T_SWCLK | SWD |
| PB3 | T_SWO | SWO trace |

---

## New Pins — Needs IOC Config

### CAN Bus (FDCAN1)
| Pin | Label | AF | Function |
|-----|-------|----|----------|
| PA11 | FDCAN1_RX | AF9 | CAN Bus receive |
| PA12 | FDCAN1_TX | AF9 | CAN Bus transmit |

**IOC config (done):**
- Prescaler=5, TimeSeg1=25, TimeSeg2=8, SJW=**2** (change from 1→2 in IOC)
- Mode: Classic CAN, Normal
- External transceiver: TJA1050 → TX→TXD, RX←RXD

**Code (done):**
- `hfdcan1` declared globally by CubeMX
- Global filter (accept-all → FIFO0) + `HAL_FDCAN_Start()` in USER CODE section
- `CAN_NODE_ID = 0x001` in `main.h`

---

## Free Pins (available for new PCB)

| Pin | Notes |
|-----|-------|
| PA10 | GPIO or USART1_RX (AF7) |
| PA11 | **→ assigned FDCAN1_RX** |
| PA12 | **→ assigned FDCAN1_TX** |
| PA15 | GPIO or FDCAN3_TX (AF9) |
| PB1 | GPIO |
| PB2 | GPIO |
| PB6 | GPIO or USART1_TX (AF7) |
| PB8 | GPIO |
| PB9 | GPIO |
| PB12 | GPIO or FDCAN2_RX (AF9) |
| PB13 | GPIO or FDCAN2_TX (AF9) — freed (was old ESTOP) |
| PB15 | GPIO |
| PC0 | GPIO |
| PC3 | GPIO |
| PC5 | USART1_RX (AF7) or GPIO |
| PC9 | GPIO |
| PC12 | GPIO |
| PD2 | GPIO |

---

## IOC Change Checklist (do before next build)

- [ ] **PA1** REED_UP: `GPIO_PULLUP` → `GPIO_PULLDOWN`
- [ ] **PA4** REED_DOWN: `GPIO_PULLUP` → `GPIO_PULLDOWN`
- [ ] **PB0** REED_GRIP: `GPIO_PULLUP` → `GPIO_PULLDOWN`
- [ ] **FDCAN1**: Add peripheral, assign PA11/PA12, configure 1Mbit/s
- [ ] **UART4**: Add peripheral, assign PB8/PB9, configure 115200 8N1
- [ ] After code-gen: uncomment `extern hfdcan1` and `extern huart4` in `main.c`

---

## Interrupt Priority Table

| Priority | IRQ | Source |
|----------|-----|--------|
| 0 | EXTI9_5 | PB5 MODE button |
| 2 | TIM7 | 1kHz control loop |
| 3 | LPUART1, USART3, DMA1_Ch1-3 | Communication |
| 15 | TIM2 | HAL timebase |
