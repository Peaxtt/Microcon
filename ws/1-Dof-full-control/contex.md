# 🤖 Project Context: 1-DOF Robot Control System
**Microcontroller:** STM32G474RE (Nucleo-G474RE)
**Framework:** STM32 HAL library (C/C++)
**Motor Driver:** Cytron MD20A (PWM + DIR control)
**Encoder:** CUI AMT102-V (Incremental Encoder, 5V)

## 📌 Hardware Abstraction & Pin Mapping
Please use the following exact pin mappings when generating initialization code, macros, or logic. 

### 1. Motor Control (Outputs)
* `PA6`  -> Motor PWM (Requires Timer PWM output configuration)
* `PA1`  -> Motor Direction (GPIO Output)

### 2. Encoder (Inputs - Hardware Timer)
* `PA8`  -> Encoder Phase B (TIM1_CH1)
* `PA9`  -> Encoder Phase A (TIM1_CH2)

### 3. Sensors & Buttons (Digital Inputs)
*Note: All 24V external signals pass through an Optocoupler (PC817) and step down to 3.3V before entering the STM32 pins.*
* `PB13` -> E-Stop Button Input
* `PB5`  -> Proximity / Home Sensor Input
* `PB1`  -> Reset Button Input *(Replaced Limit Switch)*
* `PB10` -> Power Button Input
* `PC0`  -> Mode Button Input
* *(Optional)* `PA0` -> Test Station: Reed Switch Up
* *(Optional)* `PA4` -> Test Station: Reed Switch Down
* *(Optional)* `PB0` -> Test Station: Reed Switch Gripper

### 4. Relay Control (Digital Outputs)
*Note: These pins send 3.3V signals to an 8-channel Relay board (Active-High/Low depending on relay module spec) to control 24V loads.*
* `PC7`  -> Tower Light: Green
* `PB7`  -> Tower Light: Yellow
* `PC8`  -> Tower Light: Red
* `PC1`  -> Pneumatic Cylinder: Up/Down (NO = Up, NC = Down)
* `PB11` -> Gripper: Close/Open (NO = Close, NC = Open)
* `PB4`  -> Reset Button LED Indicator
* `PB14` -> System Power Latch / Power Off Signal

### 5. Analog Sensors (ADC Input)
* `PC5`  -> Current Sensor Input (ADC Channel)

---

## 🛠️ Instructions for AI Assistant:
1. When providing code, start with the pin initialization configurations (GPIO, TIM, ADC) based on the mapping above.
2. Assume the system uses STM32CubeIDE / HAL functions (e.g., `HAL_GPIO_WritePin`, `HAL_TIM_PWM_Start`, `HAL_TIM_Encoder_Start`).
3. I will provide the specific operational logic (State Machine, PID control, interrupt handling) in the subsequent prompts. Please acknowledge this context and wait for my logic instructions.



# 🤖 Project: 1-DOF Robot Control System (Full Context)

**Project Owner:** Group 11  
**Hardware:** Nucleo-G474RE (STM32G4), Cytron MD20A, CUI AMT102-V Encoder  
**Architecture:** 3-Layer Control (Logic -> Interface Board -> Power)

---

## 1. ⚙️ Hardware & Pin Mapping (STM32 HAL)

### 🔴 Motor Control (Outputs)
* **PA6:** Motor PWM (Timer Output - PWM Mode)
* **PA1:** Motor Direction (GPIO Output: HIGH = CCW, LOW = CW)

### 🔵 Encoder Feedback (Hardware Timer)
* **PA9:** Encoder Phase A (TIM1_CH2) - Yellow Wire
* **PA8:** Encoder Phase B (TIM1_CH1) - Green Wire
* *Note: Use 5V logic for Encoder.*

### 🟢 Digital Inputs (ผ่าน Opto-Isolator 24V -> 3.3V)
* **PB13:** Emergency Stop (E-Stop)
* **PB5:** Proximity Sensor (Home Position)
* **PB1:** Reset Button (Physical Button on Control Box)
* **PB10:** Power Button
* **PC0:** Mode Selection Button

### 🟡 Relay Control (8-Channel Relay Module)
* **PC7:** Tower Light (Green)
* **PB7:** Tower Light (Yellow)
* **PC8:** Tower Light (Red)
* **PC1:** Pneumatic Cylinder (Up/Down) -> NO: Up, NC: Down
* **PB11:** Pneumatic Gripper (Close/Open) -> NO: Close, NC: Open
* **PB4:** Reset Button LED Indicator
* **PB14:** Power Latch / System Shutdown Signal (Start Button Latch)

### 🟣 Analog Signals
* **PC5:** Current Sensor (ADC Input for Stall Detection)

---

## 2. 🕹️ Joystick Mapping Logic (Control Scheme)

| Group | Buttons | Action / Function |
| :--- | :--- | :--- |
| **System** | `LT + X` | **Homing:** Search for home position using Proximity sensor |
| | `LT + Y` | **Calibration:** Reset internal position counts |
| | **`LB`** | **Software E-Stop:** Immediate PWM Cut-off |
| | **`Back`** | **Reset Alarm:** Clear Error state after E-Stop/Limit |
| **Semi-Auto**| `A` | **Pick:** Down -> Close Gripper -> Up |
| | `B` | **Place:** Down -> Open Gripper -> Up |
| **Manual** | `RT + L-Analog`| **Free Move:** Smooth arm rotation (Deadman Switch) |
| | `D-Pad ◄ / ►` | **Jog L/R:** Step-by-step small movement |
| | `D-Pad ▲ / ▼` | **Z-Axis:** Pneumatic Up/Down |
| | **`RB`** | **Gripper:** Toggle Close/Open |
| | `Start` | **Speed Toggle:** Switch between 10% / 50% / 100% Speed |

---

## 3. ⚡ Electrical Architecture & Safety Rules

* **Grounding (Star Ground):** * GND of PSU 24V and PSU 5V are connected via a **4mm² heavy-duty wire**.
    * Motor Driver (Cytron) MUST connect GND directly to 24V PSU.
    * STM32 Logic GND MUST connect to the 5V GND Bar.
    * **CRITICAL:** Do NOT connect a secondary small GND wire between STM32 and Cytron PWM/DIR pins to avoid Ground Loops.
* **Power Source (Jumper JP5):**
    * Jumper JP5 is set to **`E5V`**. The board is powered by the custom PCB (External 5V).
    * Note: ST-LINK requires the Control Box to be powered ON to communicate via USB.
* **Signal Integrity:**
    * Encoder (5V) and Proximity (24V) share an 8-core cable. 
    * Shielded cable is used for the Encoder to prevent EMI from the motor.

---

## 🛠️ Instructions for AI Assistant:
1.  **Code Generation:** Always use STM32 HAL Library.
2.  **State Machine:** The system should have states: `INIT`, `IDLE`, `HOMING`, `RUNNING`, `ERROR`, `EMERGENCY`.
3.  **Safety:** Implement a "Stall Detection" logic using ADC (PC5) to stop the motor if the current exceeds a safe threshold.
4.  **Homing Logic:** On Homing, the motor moves slowly until `PB5` (Proximity) is triggered, then sets the Encoder count to zero.
5.  **Joystick Handling:** Assume Joystick data is received via Serial/UART. Parse the button states according to the Mapping in Section 2.

**Acknowledge this context and wait for my next instruction to write specific functions.**