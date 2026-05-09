# 🤖 Project Context: 1-DOF Robot Control System
**Microcontroller:** STM32G474RE (Nucleo-G474RE)
**Framework:** STM32 HAL library (C/C++)
**Motor Driver:** Cytron MD20A (PWM + DIR control)
**Encoder:** CUI AMT102-V (Incremental Encoder, 5V)

## 📌 Hardware Abstraction & Pin Mapping
Please use the following exact pin mappings when generating initialization code, macros, or logic. 

### 1. Motor Control (Outputs)
* `PC6`  -> Motor PWM (TIM3_CH1)
* `PA0`  -> Motor Direction (GPIO Output)

### 2. Encoder (Inputs - Hardware Timer)
* `PA8`  -> Encoder Phase B (TIM1_CH1)
* `PA9`  -> Encoder Phase A (TIM1_CH2)

### 3. Sensors & Buttons (Digital Inputs)
*Note: All 24V external signals pass through an Optocoupler (PC817) and step down to 3.3V before entering the STM32 pins.*
* `PB13` -> E-Stop Button Input (EXTI)
* `PB5`  -> Mode Selector Switch (Auto/Manual)
* `PB1`  -> Reset Button Input
* `PB10` -> Power Button Input
* `PA1`  -> Reed Switch Up
* `PA4`  -> Reed Switch Down
* `PB0`  -> Reed Switch Gripper

### 4. Relay Control (Digital Outputs)
*Note: These pins send 3.3V signals to an 8-channel Relay board to control 24V loads.*
* `PC7`  -> Tower Light: Green
* `PB7`  -> Tower Light: Yellow
* `PC8`  -> Tower Light: Red
* `PC1`  -> Pneumatic Cylinder: Up/Down
* `PB11` -> Gripper: Close/Open
* `PB4`  -> Reset Button LED Indicator
* `PB14` -> System Power Latch / Power Off Signal
* `PB6`  -> Emergency Output Signal (EMER_OUT)

### 5. Analog Sensors (ADC Input)
* `PC5`  -> Current Sensor Input (ADC2_IN11)

---

## 🕹️ Joystick Mapping Logic (Control Scheme)

| Group | Buttons | Action / Function |
| :--- | :--- | :--- |
| **System** | `LT + X` | **Homing:** Search for home position |
| | `LT + Y` | **Calibration:** Reset internal position counts |
| | **`L3 + R3`** | **Software E-Stop:** Immediate PWM Cut-off |
| | **`Back`** | **Reset Alarm:** Clear Error state after E-Stop |
| **Semi-Auto**| `A` | **Pick:** Down -> Close Gripper -> Up |
| | `B` | **Place:** Down -> Open Gripper -> Up |
| **Manual** | `RT + L-Analog`| **Free Move:** Smooth arm rotation (Deadman Switch) |
| | **`Y`** | **Pneumatic Toggle** |
| | **`X`** | **Reset LED Indicator Test** |

---

## 🛠️ Instructions for AI Assistant:
1. When providing code, start with the pin initialization configurations (GPIO, TIM, ADC) based on the mapping above.
2. Assume the system uses STM32CubeIDE / HAL functions (e.g., `HAL_GPIO_WritePin`, `HAL_TIM_PWM_Start`, `HAL_TIM_Encoder_Start`).
3. The system should have states: `INIT`, `IDLE`, `HOMING`, `RUNNING`, `ERROR`, `EMERGENCY`.
