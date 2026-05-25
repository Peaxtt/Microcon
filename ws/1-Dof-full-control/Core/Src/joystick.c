#include "joystick.h"

static JoyState_t        joy = {0};
static UART_HandleTypeDef *hjoy_uart;

static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc ^= data[i];
  return crc;
}

void joystick_init(UART_HandleTypeDef *huart) {
  hjoy_uart = huart;
}

// Called from HAL_UARTEx_RxEventCallback with a raw DMA buffer slice
void joystick_parse(uint8_t *pkt) {
  if (pkt[0]  != 0xAA)              return;
  if (pkt[15] != 0x55)              return;
  if (crc8(&pkt[1], 13) != pkt[14]) return;

  joy.lx          = (int16_t)((pkt[1]  << 8) | pkt[2]);
  joy.ly          = (int16_t)((pkt[3]  << 8) | pkt[4]);
  joy.rx          = (int16_t)((pkt[5]  << 8) | pkt[6]);
  joy.ry          = (int16_t)((pkt[7]  << 8) | pkt[8]);
  joy.buttons     = (uint16_t)((pkt[9] << 8) | pkt[10]);
  joy.lt          = pkt[11];
  joy.rt          = pkt[12];
  joy.joy_present = pkt[13] & 0x01;
  joy.rp2040_alive = 1;
  joy.last_tick   = HAL_GetTick();
}

// Returns 1 if RP2040 is alive (packets arriving within JOY_TIMEOUT_MS)
uint8_t joy_rp2040_alive(void) {
  if (joy.rp2040_alive &&
     (HAL_GetTick() - joy.last_tick > JOY_TIMEOUT_MS)) {
    joy.rp2040_alive = 0;
    joy.joy_present  = 0;
  }
  return joy.rp2040_alive;
}

// Returns 1 only when gamepad is physically connected AND RP2040 is alive
uint8_t joy_is_connected(void) {
  return joy_rp2040_alive() && joy.joy_present;
}

static int16_t _dz(int16_t v) {
  return (v > -DEADZONE && v < DEADZONE) ? 0 : v;
}

int16_t joy_lx(void) { return joy_is_connected() ? _dz(joy.lx) : 0; }
int16_t joy_ly(void) { return joy_is_connected() ? _dz(joy.ly) : 0; }
int16_t joy_rx(void) { return joy_is_connected() ? _dz(joy.rx) : 0; }
int16_t joy_ry(void) { return joy_is_connected() ? _dz(joy.ry) : 0; }

float joy_lx_f(void) { return joy_lx() / 32767.0f; }
float joy_ly_f(void) { return joy_ly() / 32767.0f; }
float joy_rx_f(void) { return joy_rx() / 32767.0f; }
float joy_ry_f(void) { return joy_ry() / 32767.0f; }

uint8_t joy_lt(void) { return joy_is_connected() ? joy.lt : 0; }
uint8_t joy_rt(void) { return joy_is_connected() ? joy.rt : 0; }

float joy_lt_f(void) { return joy_lt() / 255.0f; }
float joy_rt_f(void) { return joy_rt() / 255.0f; }

uint8_t joy_btn(uint16_t mask) {
  return joy_is_connected() ? ((joy.buttons & mask) != 0) : 0;
}

uint16_t joy_raw_buttons(void) {
  return joy_is_connected() ? joy.buttons : 0;
}
