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

void modbus_init(ModbusSlave_t *slave, UART_HandleTypeDef *huart) {
  slave->huart = huart;
  memset(slave->registers, 0, sizeof(slave->registers));
  slave->rx_len = 0;
  slave->receiving = 0;
  slave->frame_ready = 0;
  slave->last_rx_time = 0;
  
  // Start DMA reception using Idle Line detection
  HAL_UARTEx_ReceiveToIdle_DMA(slave->huart, slave->rx_buf, MODBUS_BUF_SIZE);
}

// Called by TIM7 every 1ms
void modbus_tick_1ms(ModbusSlave_t *slave) {
  if (slave->receiving) {
    slave->last_rx_time++;
    // T3.5 timeout for 19200 is ~1.75ms. We use 3ms to be safe and clear buffer if stuck
    if (slave->last_rx_time > 3) { 
      slave->receiving = 0;
      slave->rx_len = 0; // Reset buffer if timeout without idle interrupt
    }
  }
}

// Called by HAL_UARTEx_RxEventCallback
void modbus_rx_cplt(ModbusSlave_t *slave, uint16_t Size) {
  slave->rx_len = Size;
  slave->receiving = 0;
  slave->frame_ready = 1;
  slave->dbg_frames_rx++;
}

static void modbus_send_response(ModbusSlave_t *slave, uint16_t len) {
  uint16_t crc = modbus_crc(slave->rx_buf, len);
  slave->rx_buf[len++] = crc & 0xFF;
  slave->rx_buf[len++] = (crc >> 8) & 0xFF;
  HAL_UART_Transmit_DMA(slave->huart, slave->rx_buf, len);
}

// Called in the 100Hz main loop
void modbus_process(ModbusSlave_t *slave) {
  if (slave->frame_ready) {
    slave->frame_ready = 0;
    
    if (slave->rx_len >= 8) {
      if (slave->rx_buf[0] == MODBUS_SLAVE_ID) {
        uint16_t crc_calc = modbus_crc(slave->rx_buf, slave->rx_len - 2);
        uint16_t crc_rx = slave->rx_buf[slave->rx_len - 2] | (slave->rx_buf[slave->rx_len - 1] << 8);
        
        if (crc_calc == crc_rx) {
          slave->dbg_crc_ok++;
          uint8_t func = slave->rx_buf[1];
          uint16_t addr = (slave->rx_buf[2] << 8) | slave->rx_buf[3];
          uint16_t val = (slave->rx_buf[4] << 8) | slave->rx_buf[5];

          if (func == 0x03) { // Read Holding Registers
            if (addr + val <= MODBUS_REG_COUNT) {
              uint16_t res_len = 3;
              slave->rx_buf[2] = val * 2;
              for (uint16_t i = 0; i < val; i++) {
                slave->rx_buf[res_len++] = (slave->registers[addr + i] >> 8) & 0xFF;
                slave->rx_buf[res_len++] = slave->registers[addr + i] & 0xFF;
              }
              modbus_send_response(slave, res_len);
            }
          } else if (func == 0x06) { // Write Single Register
            if (addr < MODBUS_REG_COUNT) {
              slave->registers[addr] = val;
              slave->dbg_reg_writes++;
              modbus_send_response(slave, 6);
            }
          }
        }
      }
    }
    
    // Restart DMA for next frame
    HAL_UARTEx_ReceiveToIdle_DMA(slave->huart, slave->rx_buf, MODBUS_BUF_SIZE);
  }
}
