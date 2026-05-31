#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

#define MODBUS_SLAVE_ID 21
#define MODBUS_BUF_SIZE 256
#define MODBUS_REG_COUNT 64   // 0x00..0x3F — custom regs up to 0x3F

typedef struct {
  uint16_t registers[MODBUS_REG_COUNT];
  UART_HandleTypeDef *huart;
  uint8_t rx_buf[MODBUS_BUF_SIZE];
  uint16_t rx_len;

  uint32_t last_rx_time;
  uint8_t receiving;
  uint8_t frame_ready;

  // Diagnostics — watch in Live Expressions to trace dropped frames
  uint32_t dbg_frames_rx;    // frames received (frame_ready fired)
  uint32_t dbg_crc_ok;       // frames that passed CRC + slave-ID check
  uint32_t dbg_reg_writes;   // FC06 register writes actually applied
  uint32_t dbg_errors;       // DMA re-arm from error callback
} ModbusSlave_t;

void modbus_init(ModbusSlave_t *slave, UART_HandleTypeDef *huart);
void modbus_tick_1ms(ModbusSlave_t *slave); // Call this in TIM7 1kHz interrupt
void modbus_process(ModbusSlave_t *slave);  // Call this in the 100Hz main loop
void modbus_rx_cplt(ModbusSlave_t *slave, uint16_t Size); // Call in UART Idle Callback

#endif /* __MODBUS_H */
