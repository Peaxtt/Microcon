# Joystick Mapping & Control Logic Proposal

Based on the updated `contex.md` guidelines, here is the mapping structure that integrates the Joystick actions seamlessly with the Modbus RTU state machine and UI requirements.

## Modbus Register Mapping Integration

The Base System UI reads from and writes to specific registers to control the robot's mode and actions. Our goal is to map joystick buttons to these internal actions, effectively simulating UI commands locally when the joystick is used.

### Mode Control (`0x01` WRITE Register equivalent)
*   The system has states: `INIT`, `IDLE`, `HOMING`, `RUNNING`, `ERROR`, `EMERGENCY`.
*   The joystick can trigger state transitions.

### 🕹️ Proposed Joystick Mapping

#### 1. System Controls
*   **`LT + X` (Homing):**
    *   **Action:** Transitions state to `HOMING`.
    *   **Logic:** Motor moves slowly until the Home Sensor (`PB5`) is triggered. Once triggered, it sets the Encoder count (TIM1) to 0.
    *   **Modbus Link:** Acts like writing `1` to `0x01` (Go Home).
*   **`LT + Y` (Calibration):**
    *   **Action:** Resets internal position counts manually without moving.
    *   **Logic:** Resets TIM1 counter.
    *   **Modbus Link:** Acts like writing `8` to `0x01` (Set Home).
*   **`LB` (Software E-Stop):**
    *   **Action:** Immediate PWM Cut-off. Transitions state to `EMERGENCY`.
    *   **Logic:** Sets TIM3 CCR to 0. Turns on Red Tower Light (`PC8`).
    *   **Modbus Link:** Acts like writing `1` to `0x25` (Soft Stop).
*   **`Back / Select` (Reset Alarm):**
    *   **Action:** Clears `ERROR` or `EMERGENCY` state (if physical E-stop `PB13` is released).
    *   **Logic:** Transitions back to `IDLE`.

#### 2. Semi-Auto Actions
*   **`A` (Pick Sequence):**
    *   **Action:** Executes a Pick sequence: Down -> Close Gripper -> Up.
    *   **Modbus Link:** Acts like writing `1` to `0x03` (Gripper Sequence: Pick).
*   **`B` (Place Sequence):**
    *   **Action:** Executes a Place sequence: Down -> Open Gripper -> Up.
    *   **Modbus Link:** Acts like writing `2` to `0x03` (Gripper Sequence: Place).

#### 3. Manual Controls (Only active in `IDLE` or `RUNNING` (Manual) state)
*   **`RT + L-Analog` (Free Move / Deadman Switch):**
    *   **Action:** Smooth arm rotation.
    *   **Logic:** The motor only moves if `RT` (Right Trigger) is held down. The speed and direction are controlled by the Left Analog Y-axis (`LY`).
    *   **Modbus Link:** This overrides Modbus P2P commands.
*   **`D-Pad ◄ / ►` (Jog L/R):**
    *   **Action:** Step-by-step small movement.
    *   **Logic:** Moves the motor a fixed number of encoder ticks (or degrees) per press.
    *   **Modbus Link:** Acts like writing to `0x05` (Jog).
*   **`D-Pad ▲ / ▼` (Z-Axis / Pneumatic):**
    *   **Action:** Pneumatic Up/Down.
    *   **Logic:** Toggles the `PC1` relay (NO = Up, NC = Down).
    *   **Modbus Link:** Acts like writing to `0x02` (Manual Gripper Motion).
*   **`RB` (Gripper Toggle):**
    *   **Action:** Toggles Close/Open state of the pneumatic gripper.
    *   **Logic:** Toggles the `PB11` relay.
*   **`Start` (Speed Toggle):**
    *   **Action:** Cycles maximum speed multiplier (10% -> 50% -> 100% -> 10%).
    *   **Logic:** Applies a multiplier to the PWM output calculated from the analog stick or jog commands.

---

## 🚦 Internal State Machine Logic

To handle both Modbus and Joystick gracefully, the main loop should implement a state machine:

1.  **`INIT`**: Setup peripherals, initial relay states (Pneumatic UP, Gripper OPEN). Move to `IDLE`.
2.  **`IDLE`**: Waiting for commands (Modbus or Joystick). Modbus heartbeat is running.
3.  **`HOMING`**: Executing home search. Ignores manual commands until done or E-Stopped.
4.  **`RUNNING`**: Executing a P2P move, Pick/Place sequence, or Manual Free Move.
5.  **`ERROR`**: Triggered by Stall Detection (Current Sensor ADC high). Requires `Back` button or Modbus reset to clear.
6.  **`EMERGENCY`**: Triggered by `LB` or physical E-Stop button (`PB13`). Requires physical reset AND software reset.

This structure ensures safety and predictable behavior regardless of whether the user is clicking on the Web UI or holding the joystick.
