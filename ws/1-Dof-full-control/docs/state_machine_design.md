# State Machine Design & Control Structure

To make the system bulletproof and clear, we are defining strict **Stages (States)**. The robot can only be in ONE state at a time. Each state defines *what inputs it listens to* and *what actions it can perform*. 

The **Power Button (`PB10`)** is a special case: it is polled constantly in the 1kHz Control Loop, completely independent of the State Machine. If it is held for 3 seconds, it shuts down the system (`PB14` goes LOW) regardless of the current state.

---

## 🧭 Core States

### 1. `STATE_IDLE`
*   **Description:** The default state when the system boots or finishes a task. The motor holds its current position (PWM active but speed = 0, or PID holding).
*   **Allowed Actions:**
    *   Transition to `STATE_CALIBRATE` (via Joystick `LT+X` or Modbus).
    *   Transition to `STATE_MANUAL` (via Modbus Mode Select or Joystick analog movement).
    *   Transition to `STATE_AUTO` (via Modbus Mode Select).
*   **Ignored Actions:** Sequences, P2P targets.

### 2. `STATE_CALIBRATE` (Homing)
*   **Description:** The robot is actively searching for the Home sensor (`PB5`).
*   **Allowed Actions:**
    *   Move motor slowly in the homing direction.
    *   If `PB5` triggers -> Reset Encoder to 0, Transition to `STATE_IDLE`.
    *   If `LB` or `E-Stop` pressed -> Transition to `STATE_EMER`.
*   **Ignored Actions:** Joystick analog input, Modbus Pick/Place commands.

### 3. `STATE_MANUAL` (Jog & Free Move)
*   **Description:** The user has direct control over the robot via the Joystick or Modbus Jog buttons.
*   **Allowed Actions:**
    *   `RT + L-Analog` to move freely (with deadband applied).
    *   D-Pad Left/Right for discrete Jog steps.
    *   Relay toggles (Gripper, Pneumatic).
    *   Transition to `STATE_IDLE` when no manual input is detected for a timeout, or mode is switched.
    *   Transition to `STATE_EMER` if E-Stop pressed.
*   **Ignored Actions:** Auto sequences.

### 4. `STATE_AUTO` (Pick & Place / P2P)
*   **Description:** The robot is executing a pre-programmed sequence from Modbus (`0x12 - 0x22`) or a Point-to-Point move.
*   **Allowed Actions:**
    *   Move motor to target positions using PID.
    *   Automatically toggle Gripper/Pneumatic according to the sequence.
    *   Transition to `STATE_IDLE` when sequence is complete.
    *   Transition to `STATE_EMER` if E-Stop pressed or Soft Stop (`0x25`) triggered.
*   **Ignored Actions:** Joystick analog input, Manual relay toggles.

### 5. `STATE_EMER` (Emergency)
*   **Description:** Triggered by Hardware E-Stop (`PB13`), Software E-Stop (`LB` on Joystick), or Analog Watchdog (Motor Overcurrent).
*   **Allowed Actions:**
    *   **CRITICAL:** Motor PWM is forced to 0 immediately.
    *   Red Tower Light (`PC8`) is turned ON.
    *   Transition back to `STATE_IDLE` **ONLY IF** the physical E-Stop is released **AND** the Reset Alarm button (`Back` on Joystick or Modbus Reset) is pressed.
*   **Ignored Actions:** ALL movement commands.

---

## ⚡ Global Overrides (Running in 1kHz TIM7 Interrupt)

These functions execute every 1 millisecond, regardless of the State Machine:

1.  **Stall Detection (AWD / ADC):** Checks if the current exceeds the safety threshold. If yes -> Force `STATE_EMER`.
2.  **Power Shutdown (Power Latch):** Checks `PB10`. If held continuously for 3000ms -> Set `PB14` LOW (Cut power).
3.  **PID Control Update:** Calculates the motor PWM required to reach the target position (if in Auto or holding position).
4.  **100Hz Sub-Loop (Software Counter):** Every 10 ticks (10ms), it triggers the Modbus register updates, Joystick parsing, and UI communication to keep the CPU load light.

---

## 🧠 What's Next?
Please review these states. If this structure looks perfect for your team, we can start coding the `C` logic for this state machine!