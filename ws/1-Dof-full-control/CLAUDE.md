# 1-DOF Robot Firmware — CLAUDE.md
> Context for Claude Code sessions. Last updated: 2026-05-25

## What this project is
STM32G474RE firmware for a 1-DOF pick-and-place robot arm (lab project, FRA263/264).
- Dev board: NUCLEO-G474RE
- Motor: DC with quadrature encoder (8192 counts/rev), Cytron MDD3A driver
- Joystick: Xbox → RP2040-Zero → USART3 @ 460800 8N1
- PC interface: Modbus RTU (LPUART1, 19200 8E1, Slave ID=21) via BaseSystem .exe

## Key file
**`Core/Src/main.c`** — all application logic (~2460 lines)
Everything important lives here: state machine, control loop, Modbus, trajectory, tower lights.

## Architecture
```
TIM7 ISR @ 1 kHz:
  Encoder → windowed velocity (10-sample) → IIR filter
  → Position PID (outer) → vel setpoint
  → Velocity PID (inner) → PWM output

Main loop @ 100 Hz (flag_10ms):
  ESTOP poll → Reed → ADC → Modbus → State machine → Tower lights
```

## State machine
`STATE_INIT → STATE_IDLE → STATE_HOMING_FAST → STATE_HOMING_BACKOFF → STATE_HOMING_SLOW → STATE_IDLE(homed) → STATE_AUTO / STATE_MANUAL`
`STATE_EMER` triggered by ESTOP LOW (200ms debounce). Exit: release ESTOP + hold RESET 50ms.

## Homing (two-pass)
1. HOMING_FAST: direct PWM left 25%
2. HOME_SENSOR (PC3, PULLUP, EXTI3 rising) triggers → HOMING_BACKOFF
3. HOMING_BACKOFF: direct PWM right 20% for 100 ticks (1s) — **no PID**
4. HOMING_SLOW: direct PWM left 6%
5. Second sensor trigger → `finish_homing()` → `cumulative_angle_deg=0`, `homed=1`

Key defaults: `encoder_inverted=1`, `homing_backoff_speed=0.20f`, `homing_backoff_ticks=100`

## DevDashboard (Live Expressions)
`DevDashboard_t dev_dash` has **6 sub-structs** (folders in Live Expressions):
| Folder | Contents |
|--------|----------|
| `dev_dash.Cmd` | target_deg, start_move, set_home, cancel_move |
| `dev_dash.Status` | pos_deg, vel_rad_s, pos_err, pwm_out, traj_active, current_A… |
| `dev_dash.Traj` | traj_type, v_max, a_max, j_max, t_acc_seg… |
| `dev_dash.Sys` | mode, max_speed, ramp_rate, acc_alpha… |
| `dev_dash.IO` | in_estop, out_tower_g, joy_connected, joy_ly… |
| `dev_dash.Test` | force_motor, test_speed, test_period_fwd_ms… |

PID gains are **separate globals** in `pid_control.c`: `kp_vel=30`, `ki_vel=0.1`, `kd_vel=0.0`, `kp_pos=1.0`
To tune: edit via Live Expressions OR use Modbus 0x0C-0x0F.

## Recent changes (2026-05-25 → 2026-05-26)
- ESTOP debounce: 200ms solid LOW required before EMER triggers (`estop_debounce`, 20 ticks)
- RESET debounce: 50ms hold required to clear EMER (`emer_rel_cnt`, 5 ticks)
- HOMING_BACKOFF: replaced PID with direct PWM right (`homing_backoff_speed = 0.20f`)
- `encoder_inverted` default changed to `1`
- `dev_dash` refactored to 6 sub-structs (143 references renamed)
- MOTOR_DIR moved PC8 → PC7, REED_UP moved PA1 → PA7 (PCB test results)
- EMER_OUTPUT moved PC6 → PC11 (cleaner routing)
- USART3 joystick moved PC10/PC11 → PB8/PB9 (PCB layout)
- USART3 baud set to 460800, DMA1 Ch4 RX added (manual until IOC updated)

## Pin highlights
| Pin | Function |
|-----|----------|
| PC9 | PWM (TIM3 CH4, 20kHz) |
| PA7 | REED_UP (PULLDOWN, active HIGH) |
| PA8/PA9 | ENC_A/B (TIM1 TI12) |
| PA15 | ESTOP (PULLUP, LOW=active) |
| PC3 | HOME_SENSOR (PULLUP, EXTI3 rising, HIGH=detected) |
| PC7 | MOTOR_DIR (Output) |
| PC11 | EMER_OUTPUT (Output) |
| PC13 | RESET_BTN (PULLUP, LOW=active) |
| PA2/PA3 | LPUART1 Modbus |
| PB8/PB9 | USART3 Joystick RX/TX |

## Build & flash
STM32CubeIDE: **Ctrl+B** to build, **F11** to flash. If IDE shows main.c unsaved after external edits, press **F5** on the file to reload from disk first.

## Other key files
- `Core/Inc/pid_control.h` + `Core/Src/pid_control.c` — PID gains and functions
- `Core/Src/stm32g4xx_it.c` — ISR handlers (TIM7, EXTI, UART)
- `Core/Src/SCURVE.c` / `TRAPEZOID.c` — trajectory generators
- `docs/PROJECT_CONTEXT.md` — full system reference (Thai/English)
- `docs/instruction.html` — setup & operation guide
- `README.md` — control team handoff + Live Expressions reference
