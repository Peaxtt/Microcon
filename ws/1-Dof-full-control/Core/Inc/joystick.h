#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "main.h"

#define PKT_LEN  15
#define DEADZONE 3200 // ~10% Deadband to prevent motor whine

// Button defines
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
#define BTN_GUIDE      0x0400
#define BTN_A          0x1000
#define BTN_B          0x2000
#define BTN_X          0x4000
#define BTN_Y          0x8000

typedef struct {
  int16_t  lx, ly, rx, ry;
  uint16_t buttons;
  uint8_t  lt, rt;
  uint8_t  connected;
  uint32_t last_tick;
} JoyState_t;

void joystick_init(UART_HandleTypeDef *huart);
void joystick_parse(uint8_t *pkt);
uint8_t joy_is_connected(void);
int16_t joy_lx(void);
int16_t joy_ly(void);
int16_t joy_rx(void);
int16_t joy_ry(void);
float joy_lx_f(void);
float joy_ly_f(void);
float joy_rx_f(void);
float joy_ry_f(void);
uint8_t joy_lt(void);
uint8_t joy_rt(void);
float joy_lt_f(void);
float joy_rt_f(void);
uint8_t joy_btn(uint16_t mask);

#endif /* __JOYSTICK_H */
