# Cleanup Plan — Re-apply Improvements to 52632b4 Base

> Base commit: `52632b4` (modbus_fixed)  
> Reason for revert: refactored structure caused ISR/control instability; cleaner to re-apply improvements one by one.

---

## Improvements confirmed good — re-apply to main.c

### 1. Tower lights (update_tower_lights)
Replace the old static tower logic with:

| State | Light |
|-------|-------|
| ST_EMER (ESTOP pressed) | Red blink 2Hz |
| ST_EMER (ESTOP released, wait reset) | Red steady |
| ST_HOMING_FAST | Yellow blink fast 5Hz |
| ST_HOMING_BACKOFF | Yellow blink 2.5Hz |
| ST_HOMING_SLOW | Yellow blink 1Hz |
| ST_IDLE, not homed | Y↔G alternating blink (prompt to home) |
| ST_IDLE, homed | Green steady |
| ST_IDLE, just set-home | Y-Y-G flash (130 tick counter) |
| ST_MANUAL_SWITCH | Yellow steady |
| ST_MANUAL_MODBUS | Yellow steady |
| ST_AUTO | Green blink, speed proportional to velocity: >4 rad/s → 5Hz, >1 → 1.7Hz, >0.2 → slow, settled → steady |
| ST_SEQUENCE | Green blink 2Hz + Yellow steady |
| ST_TEST | Green + Yellow blink 2Hz |
| Soft limit active | Red + Green steady (overlaid) |

Tick counter: 8-bit, increments every 10ms loop, wraps at 255.

### 2. EMER_OUTPUT fail-safe polarity
- Boot: `HAL_GPIO_WritePin(EMER_OUTPUT, PIN, GPIO_PIN_SET)` (relay energized = safe)
- Enter EMER: `GPIO_PIN_RESET` (relay drops = safe-off)
- Exit EMER: `GPIO_PIN_SET` (re-energize)

### 3. Reset LED guard
Only blink reset LED if still in EMER state.  
Pattern: if ESTOP active → LED off; else → blink 2Hz.  
Clear LED immediately on EMER exit.

```c
if (current_state == STATE_EMER) {
  HAL_GPIO_WritePin(RESET_LED, PIN,
    robot_estop ? GPIO_PIN_RESET : ((tick % 50) < 25) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```

### 4. Joystick mapping (ST_MANUAL_SWITCH)
Replace old LT+A/Y pick-place with:

| Control | Action |
|---------|--------|
| RT (deadman) + LY | Motor speed, max 60% PWM, with ramp 0.03/tick |
| DPAD Left/Right | Fine tune ±`homing_slow_pct` (default 0.07 = 7%) |
| Y (edge) | Toggle cylinder up/down |
| B (edge) | Toggle gripper open/close |
| LB (edge) | Pick sequence |
| RB (edge) | Place sequence |
| RT + A (edge) | Set home (virtual zero at current position) |
| X | Enter EMER immediately |

Stick dead zone: ±0.08.  
`homing_slow_pct` is shared with homing slow approach speed AND DPAD fine tune speed.

### 5. Soft rotation limit ±540°
- Replaces hard EMER at ±720°
- When `fabsf(cumul_deg) >= 540°`: directional clamp in ISR (block movement toward limit, allow reverse)
- State machine: call `control_reset()` on first hit (not full EMER)
- Tower: R+G steady overlay while clamped
- Variable: `volatile int8_t soft_limit_dir = 0;  // 0=none +1=pos -1=neg`

```c
// In 1kHz ISR, after control_update:
if (fabsf(cumul_deg) >= 540.0f) {
  soft_limit_dir = (cumul_deg > 0) ? 1 : -1;
  if (soft_limit_dir > 0 && pwm > 0) pwm = 0;
  if (soft_limit_dir < 0 && pwm < 0) pwm = 0;
} else {
  soft_limit_dir = 0;
}
```

### 6. Pick/place gripper fix
In `SEQ_GOING_PICK` and manual pick start:
```c
hardware_set_actuator(1, 1, 0);  // DOWN + OPEN gripper before picking
// NOT: hardware_set_actuator(1, 0, 0)
```

### 7. Actuator register 0x02 — edge-detect + UP fix
Old code ignores `reg[0x02] = 0x00` because `if (r[0x02])` skips zero.  
Fix: use edge-detect on **any change** including to 0x00:

```c
static uint16_t prev_act = 0xFFFF;  // invalid sentinel
if (r[0x02] != prev_act) {
  prev_act = r[0x02];
  // decode: 0x00=UP, 0x01=DOWN, 0x02=OPEN, 0x04=CLOSE
}
```

### 8. Jog — cumulative position, correct direction
Old: `target = fmodf(cmd_jog_deg * PI/180, 2π)` — causes wrap-around runaway.  
Fix: `target = pos_rad + cmd_jog_deg * (π/180)` — always relative to current.

Modbus sign: BaseSystem +N = CCW, firmware encoder + = CW → negate on receive:
```c
jog_deg = -(float)(int16_t)r[0x05];
```

### 9. Sequence timing variables (tunable via Live Expressions)
```c
volatile float seq_dwell_ms   = 200.0f;  // min wait before accepting reed
volatile float seq_timeout_ms = 3000.0f; // force-advance if no reed
```
Convert in state machine: `dwell_ticks = seq_dwell_ms / 10` (100Hz loop).

---

## Control integration — separate task

When the control team's `control.c` is ready to integrate:

1. The refactored file structure (`robot.c/h`, `state_machine.c/h`, `hardware.c/h`, `modbus_app.c/h`, `control.h`, `control_defaults.c`) should be restored from commit `62610ae` or rebuilt.

2. **ISR motor conflict** (critical bug from the refactor):  
   State machine sets motor at 100Hz, ISR overrides at 1kHz.  
   Fix: add `volatile float robot_motor_pct = 0` as the shared variable.  
   ISR: `hardware_set_motor(robot_motor_pct)` when in homing/manual states.  
   State machine: write to `robot_motor_pct`, not `hardware_set_motor()` directly.

3. **control_model[1] zero initialization** causes runaway on long-distance moves:  
   Set `control_model_switch_deg = 9999` to always use model[0].

4. `dev_dash_sync()` must sync `DashCtrl_t` → `control_model[0]` every 100Hz using correct field names (see `LIVE_EXPRESSIONS.md`).

---

## Encoder direction diagnosis
Use `docs/robot_logger.py` before testing jog/P2P:
```
cd C:\PaYae\Microcon\ws\1-Dof-full-control\docs
python robot_logger.py COM3
```
Tracking diagnosis: if INVERTED (red background), velocity goes away from target.  
Fix: toggle `encoder_inverted` in Live Expressions (0→1) or negate PWM in ISR for control states.

---

## python_gui notes
`python_gui/main.py` (PyQt5 + pyqtgraph) reads telemetry from firmware custom protocol AND Modbus.  
Uses separate register map (reg 10–25) from BaseSystem.  
Run: `python python_gui/main.py` (requires PyQt5, pyqtgraph, pyserial, scipy, numpy).

`docs/robot_logger.py` (matplotlib) reads BaseSystem Modbus registers (reg 0x28–0x31).  
Simpler, no install besides `pip install pyserial matplotlib`.
