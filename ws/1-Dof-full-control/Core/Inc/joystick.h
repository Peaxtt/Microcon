#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "main.h"

#define PKT_LEN      16     // 16-byte packet (matches RP2040 firmware)
#define DEADZONE     3200   // ~10% — suppress motor whine at rest

// ── XInput button masks ───────────────────────────────────────────────────
#define BTN_DPAD_UP    0x0001
#define BTN_DPAD_DOWN  0x0002
#define BTN_DPAD_LEFT  0x0004
#define BTN_DPAD_RIGHT 0x0008
#define BTN_START      0x0010
#define BTN_BACK       0x0020
#define BTN_L3         0x0040
#define BTN_R3         0x0080
#define BTN_LB         0x0100
#define BTN_RB         0x0200
#define BTN_T          0x0400   // Center/Guide/T button (EGA J3 middle button)
#define BTN_A          0x1000
#define BTN_B          0x2000
#define BTN_X          0x4000
#define BTN_Y          0x8000

// ── Timeout ───────────────────────────────────────────────────────────────
#define JOY_TIMEOUT_MS  500   // ms without any packet → RP2040 considered dead

// ── State struct ──────────────────────────────────────────────────────────
typedef struct {
  int16_t  lx, ly, rx, ry;
  uint16_t buttons;
  uint8_t  lt, rt;
  uint8_t  rp2040_alive;   // 1 = receiving packets (RP2040 is running)
  uint8_t  joy_present;    // 1 = gamepad physically plugged into RP2040
  uint32_t last_tick;
} JoyState_t;

// ── API ───────────────────────────────────────────────────────────────────
void    joystick_init(UART_HandleTypeDef *huart);
void    joystick_parse(uint8_t *pkt);

uint8_t joy_is_connected(void);    // true: gamepad present AND RP2040 alive
uint8_t joy_rp2040_alive(void);    // true: packets arriving (RP2040 running)

int16_t joy_lx(void);
int16_t joy_ly(void);
int16_t joy_rx(void);
int16_t joy_ry(void);
float   joy_lx_f(void);
float   joy_ly_f(void);
float   joy_rx_f(void);
float   joy_ry_f(void);
uint8_t  joy_lt(void);
uint8_t  joy_rt(void);
float    joy_lt_f(void);
float    joy_rt_f(void);
uint8_t  joy_btn(uint16_t mask);
uint16_t joy_raw_buttons(void);

#endif /* __JOYSTICK_H */
