# 1-DOF Robot Firmware

> STM32G474RETx @ 170 MHz · NUCLEO-G474RE  
> Last updated: 2026-05-31

---

## ภาพรวม

Firmware สำหรับหุ่นยนต์ pick-and-place 1 แกน (FRA263/264)  
ดิสก์มีรูสำหรับวาง rod ทุก 5° (72 ตำแหน่ง) หมุนได้ 0–360°  
BaseSystem สั่งงานผ่าน Modbus RTU, joystick ควบคุม manual mode

## โครงสร้างไฟล์

```
Core/Src + Core/Inc:
  robot.c/.h          — shared state ทั้งหมด (globals ทุกตัว)
  hardware.c/.h       — GPIO / ADC / PWM / encoder wrapper
  modbus_app.c/.h     — Modbus register map
  state_machine.c/.h  — state machine + tower lights
  control.h           — control layer interface (ทีม control implement)
  control_defaults.c  — weak stubs (build ได้แม้ไม่มี control.c)

  modbus.c/.h         — Modbus RTU protocol driver
  joystick.c/.h       — Xbox gamepad parser
  ENCODER.c/.h        — encoder math
  main.c              — HAL init + ISR + while loop
```

## Architecture

```
1kHz ISR: Encoder → velocity filter → control_update() → motor PWM
          Hard limit ±720° → ST_EMER

100Hz loop: hardware_sensors_update()
            modbus_app_receive()   (registers → commands)
            state_machine_tick()   (states + outputs)
            modbus_app_send()      (state → registers)
```

## State hierarchy

```
ST_EMER          — ESTOP or joystick LB (universal override)
ST_MANUAL_SWITCH — cabinet switch HIGH: joystick controls motor only
  ST_IDLE / ST_HOMING_* / ST_MANUAL_MODBUS / ST_AUTO / ST_SEQUENCE / ST_TEST
  (cabinet switch LOW = Modbus controls all of the above)
```

## Control layer integration

ทีม control implement `control.c` ตาม interface ใน `control.h`:
```c
void    control_init(void);
void    control_set_target(float target_rad);
void    control_update(float pos_rad, float vel_rad_s, float *pwm_out);
void    control_reset(void);
uint8_t control_is_settled(float pos_rad);
```
รองรับ 2 โมเดล (`control_model[0]` fine, `control_model[1]` coarse)  
เลือกโมเดลอัตโนมัติจาก displacement vs `control_model_switch_deg` (default 30°)

## Modbus Register Map

ดู `Core/Inc/modbus_app.h` สำหรับ map เต็ม  
Slave ID = 21, LPUART1, 19200 8E1

| Reg | Direction | Description |
|-----|-----------|-------------|
| 0x00 | R/W | Heartbeat HI(18537) → YA(22881) |
| 0x01 | W | Mode cmd: 0x01=HOME 0x02=MANUAL 0x04=AUTO 0x08=SET_HOME 0x10=TEST |
| 0x05 | W | Jog (signed degrees, auto-clear, wraps 0–360°) |
| 0x24 | W | P2P target (degrees, edge-triggered) |
| 0x23 | R | Homed flag |
| 0x28 | R | Position 0–359.99° × 10 |
| 0x29 | R | Velocity deg/s × 10 |
| 0x2F | R | Current state enum (0–10) |
| 0x26 | R | Reed switches bit0=up bit1=down bit2=grip |
| 0x31 | R | ESTOP active |

## Build

STM32CubeIDE → **Ctrl+B** build, **F11** flash  
หลังเพิ่มไฟล์ใหม่: คลิกขวา project → **Refresh (F5)**

## หมายเหตุ

- `encoder_inverted` ตั้งผ่าน Live Expressions เท่านั้น (Modbus reg[0x36] เป็น read-only)
- Hard rotation limit ±720° enforce ใน 1kHz ISR ทุก state
- `home_offset_deg = 0.6°` — ปรับหลัง verify ตำแหน่ง home จริง
