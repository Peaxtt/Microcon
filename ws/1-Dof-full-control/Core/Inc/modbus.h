#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

#define MODBUS_SLAVE_ID 1
#define MODBUS_BUF_SIZE 256

typedef struct {
  uint16_t *registers;
  uint16_t reg_count;
  UART_HandleTypeDef *huart;
  uint8_t rx_buf[MODBUS_BUF_SIZE];
  uint16_t rx_idx;
  uint32_t last_rx_tick;
} ModbusSlave_t;

void modbus_init(ModbusSlave_t *slave, UART_HandleTypeDef *huart, uint16_t *regs, uint16_t count);
void modbus_update(ModbusSlave_t *slave);

#endif /* __MODBUS_H */
