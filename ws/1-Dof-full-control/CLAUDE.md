# 1-DOF Robot Firmware — CLAUDE.md
> Context for Claude Code sessions. Last updated: 2026-05-31

## What this project is
STM32G474RE firmware for a 1-DOF pick-and-place robot arm (FRA263/264 lab project).
Disk with rod holes every 5° (72 positions). Robot picks rod from one hole and places in another.
- Dev board: NUCLEO-G474RE
- Motor: DC with quadrature encoder (8192 counts/rev), Cytron MDD3A driver
- Joystick: Xbox → RP2040-Zero → USART3 @ 460800 8N1
- PC interface: Modbus RTU (LPUART1, 19200 8E1, Slave ID=21) via BaseSystem .exe

## File structure

### Application layer (our code)
| File | Role |
|------|------|
| `robot.c/.h` | All shared globals — single source of truth |
| `hardware.c/.h` | GPIO / ADC / PWM / encoder abstraction |
| `modbus_app.c/.h` | Modbus register map (receive commands + send feedback) |
| `state_machine.c/.h` | State machine + tower lights |
| `control.h` | Control layer interface (team implements `control.c`) |
| `control_defaults.c` | `__weak` stubs so project builds without `control.c` |

### Driver layer (don't modify)
`modbus.c/.h`, `joystick.c/.h`, `ENCODER.c/.h`

### STM-generated (never touch)
`stm32g4xx_*`, `syscalls.c`, `sysmem.c`, `system_stm32g4xx.c`, `main.h`

## Architecture

```
1kHz ISR (TIM7):
  Encoder_Update → robot_pos_rad, robot_vel_rad_s, robot_cumul_deg
  Hard limit: |robot_cumul_deg| >= 720° → ST_EMER immediately
  control_update(pos, vel, &pwm) → hardware_set_motor(pwm)

100Hz main loop (flag_10ms):
  hardware_sensors_update()   <- read all sensors -> robot_* globals
  modbus_app_receive()        <- registers -> commands
  state_machine_tick()        <- state transitions + actuators + tower lights
  modbus_app_send()           <- robot state -> registers
```

## State machine hierarchy

```
ST_EMER          <- ESTOP LOW (debounce 20 ticks) OR joystick LB (any state)
                    exit: release ESTOP + hold RESET 50ms

ST_MANUAL_SWITCH <- cabinet switch HIGH: joystick controls motor, Modbus blocked

(cabinet switch LOW -- Modbus controls):
  ST_INIT -> ST_IDLE
  ST_HOMING_FAST -> ST_HOMING_BACKOFF -> ST_HOMING_SLOW -> ST_IDLE
  ST_MANUAL_MODBUS  (reg[0x01]=0x02) -- jog + pneumatics
  ST_AUTO           (reg[0x01]=0x04, requires homed) -- P2P
  ST_SEQUENCE       (reg[0x04] autostart) -- pick/place loop
  ST_TEST           (reg[0x01]=0x10) -- test mode
```

## Control layer (implemented by control team)
```c
// control.h -- team implements these in control.c
void    control_init(void);
void    control_set_target(float target_rad);
void    control_update(float pos_rad, float vel_rad_s, float *pwm_out);  // 1kHz ISR
void    control_reset(void);
uint8_t control_is_settled(float pos_rad);
```
Multiple models: `control_model[0]` (fine/near), `control_model[1]` (coarse/far).
Switch threshold: `control_model_switch_deg` (default 30 deg).

## Key variables (all in robot.h / robot.c)
```
robot_state         -- current state (RobotState_t)
robot_homed         -- 1 = homed
robot_pos_rad       -- encoder position (rad)
robot_vel_rad_s     -- filtered velocity (rad/s)
robot_cumul_deg     -- cumulative rotation from home (safety tracking)
encoder_inverted    -- 0 or 1 -- Live Expressions only, NOT Modbus-writable
home_offset_deg     -- move after homing (default 0.6 deg)
```

## Modbus register map (abbreviated)
Full map in `modbus_app.h`. Key registers:
- 0x01 Mode cmd | 0x05 Jog deg | 0x24 P2P target | 0x19 Soft stop
- 0x28 Position x10 | 0x29 Velocity | 0x2F State enum | 0x23 Homed
- 0x26 Reed switches | 0x31 ESTOP | 0x32 Digital IO
- 0x0C-0x0F PID vel | 0x38-0x3F Traj/PID pos params (stored for control team)

## Pin highlights
| Pin | Function |
|-----|----------|
| PC9 | PWM (TIM3 CH4, 20kHz) |
| PC7 | MOTOR_DIR |
| PA8/PA9 | ENC_A/B (TIM1 TI12) |
| PA15 | ESTOP (PULLUP, LOW=active) |
| PC3 | HOME_SENSOR (PULLUP, EXTI3 rising, HIGH=detected) |
| PC2 | MODE switch (LOW=AUTO, HIGH=MANUAL) |
| PC13 | RESET_BTN (PULLUP, LOW=active) |
| PA7 | REED_UP (PULLDOWN, HIGH=active) |
| PA4 | REED_DOWN | PB0 REED_GRIP |
| PB1 | PNEUMATIC | PB2 GRIPPER |
| PB13/14/15 | TOWER_R/Y/G |
| PA0 | Current ADC (ADC2 DMA) |
| PA2/PA3 | LPUART1 Modbus TX/RX |
| PB8/PB9 | USART3 Joystick RX/TX |
| PC11 | EMER_OUTPUT relay |
| PD2 | RESET_LED |

## Build & flash
STM32CubeIDE: Ctrl+B to build, F11 to flash.
After adding new .c files: right-click project -> Refresh (F5) so CubeIDE picks them up.
New files that need to be in CubeIDE source tree:
  robot.c, hardware.c, modbus_app.c, state_machine.c, control_defaults.c
