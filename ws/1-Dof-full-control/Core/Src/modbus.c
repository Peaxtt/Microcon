#include "modbus.h"
#include <string.h>

static uint16_t modbus_crc(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void modbus_init(ModbusSlave_t *slave, UART_HandleTypeDef *huart, uint16_t *regs, uint16_t count) {
  slave->huart = huart;
  slave->registers = regs;
  slave->reg_count = count;
  slave->rx_idx = 0;
  slave->last_rx_tick = 0;
}

static void modbus_send_response(ModbusSlave_t *slave, uint16_t len) {
  uint16_t crc = modbus_crc(slave->rx_buf, len);
  slave->rx_buf[len++] = crc & 0xFF;
  slave->rx_buf[len++] = (crc >> 8) & 0xFF;
  HAL_UART_Transmit(slave->huart, slave->rx_buf, len, 100);
}

void modbus_update(ModbusSlave_t *slave) {
  uint8_t data;
  if (HAL_UART_Receive(slave->huart, &data, 1, 0) == HAL_OK) {
    if (HAL_GetTick() - slave->last_rx_tick > 5) { // T3.5 equivalent
      slave->rx_idx = 0;
    }
    if (slave->rx_idx < MODBUS_BUF_SIZE) {
      slave->rx_buf[slave->rx_idx++] = data;
    }
    slave->last_rx_tick = HAL_GetTick();
  }

  if (slave->rx_idx >= 8 && HAL_GetTick() - slave->last_rx_tick > 2) {
    if (slave->rx_buf[0] == MODBUS_SLAVE_ID) {
      uint16_t crc_calc = modbus_crc(slave->rx_buf, slave->rx_idx - 2);
      uint16_t crc_rx = slave->rx_buf[slave->rx_idx - 2] | (slave->rx_buf[slave->rx_idx - 1] << 8);
      
      if (crc_calc == crc_rx) {
        uint8_t func = slave->rx_buf[1];
        uint16_t addr = (slave->rx_buf[2] << 8) | slave->rx_buf[3];
        uint16_t val = (slave->rx_buf[4] << 8) | slave->rx_buf[5];

        if (func == 0x03) { // Read Holding Registers
          if (addr + val <= slave->reg_count) {
            uint16_t res_len = 3;
            slave->rx_buf[2] = val * 2;
            for (uint16_t i = 0; i < val; i++) {
              slave->rx_buf[res_len++] = (slave->registers[addr + i] >> 8) & 0xFF;
              slave->rx_buf[res_len++] = slave->registers[addr + i] & 0xFF;
            }
            modbus_send_response(slave, res_len);
          }
        } else if (func == 0x06) { // Write Single Register
          if (addr < slave->reg_count) {
            slave->registers[addr] = val;
            modbus_send_response(slave, 6);
          }
        }
      }
    }
    slave->rx_idx = 0;
  }
}
