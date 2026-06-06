#include "hardware.h"
/* REED_ACTIVE and HOME_SENSOR_ACTIVE are defined in main.h (included via robot.h → main.h) */

void hardware_sensors_update(void)
{
  /* ── Reed Switch State ─────────────────────────────────────────────────── */
  {
    static uint8_t  rd_prev_dummy  = 0xFF;
    static uint32_t rd_sw_timer    = 0;
    static uint8_t  last_pneu      = 0xFF, last_grip_out = 0xFF;
    static uint32_t pneu_timer     = 0,    grip_timer    = 0;
    static uint8_t  pneu_pend      = 0,    grip_pend     = 0;

    if (rd_prev_dummy == 0xFF) rd_prev_dummy = reed_dummy_en;

    if (rd_prev_dummy && !reed_dummy_en) {
      uint32_t sw_t = (uint32_t)(reed_switch_delay_ms / 10.0f);
      rd_sw_timer = (sw_t < 1) ? 1 : sw_t;
    }
    if (!rd_prev_dummy && reed_dummy_en) {
      last_pneu = 0xFF; last_grip_out = 0xFF;
      pneu_pend = 0;    grip_pend     = 0;
    }
    if (rd_sw_timer > 0) rd_sw_timer--;
    rd_prev_dummy = reed_dummy_en;
    uint8_t use_dummy = reed_dummy_en || (rd_sw_timer > 0);

    if (!use_dummy) {
      reed_up   = (HAL_GPIO_ReadPin(REED_UP_GPIO_Port,   REED_UP_Pin)   == REED_ACTIVE) ? 1 : 0;
      reed_down = (HAL_GPIO_ReadPin(REED_DOWN_GPIO_Port, REED_DOWN_Pin) == REED_ACTIVE) ? 1 : 0;
      reed_grip = (HAL_GPIO_ReadPin(REED_GRIP_GPIO_Port, REED_GRIP_Pin) == REED_ACTIVE) ? 1 : 0;
    } else {
      uint32_t delay_ticks = (uint32_t)(reed_dummy_delay_ms / 10.0f + 0.5f);
      if (delay_ticks < 1) delay_ticks = 1;

      uint8_t cur_pneu = (HAL_GPIO_ReadPin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin) == GPIO_PIN_SET) ? 1 : 0;
      uint8_t cur_grip = (HAL_GPIO_ReadPin(GRIPPER_GPIO_Port,   GRIPPER_Pin)   == GPIO_PIN_SET) ? 1 : 0;

      if (last_pneu == 0xFF) { last_pneu = cur_pneu; reed_down = cur_pneu; reed_up = !cur_pneu; }
      else if (cur_pneu != last_pneu) { last_pneu = cur_pneu; pneu_pend = 1; pneu_timer = 0; }
      if (pneu_pend && ++pneu_timer >= delay_ticks) {
        pneu_pend = 0; reed_down = cur_pneu; reed_up = !cur_pneu;
      }

      if (last_grip_out == 0xFF) { last_grip_out = cur_grip; reed_grip = cur_grip; }
      else if (cur_grip != last_grip_out) { last_grip_out = cur_grip; grip_pend = 1; grip_timer = 0; }
      if (grip_pend && ++grip_timer >= delay_ticks) {
        grip_pend = 0; reed_grip = cur_grip;
      }
    }
  }

  home_sensor_raw = (HAL_GPIO_ReadPin(HOME_SENSOR_GPIO_Port, HOME_SENSOR_Pin) == HOME_SENSOR_ACTIVE) ? 1 : 0;

  /* ── Current Sensor — average ADC2 DMA circular buffer (40 samples) ─── */
  {
    extern uint32_t adc_dma_buf[40];
    uint32_t asum = 0;
    for (int i = 0; i < 40; i++) asum += adc_dma_buf[i];
    float v = ((float)(asum / 40u) / 4095.0f) * 3.3f;
    float i_a = (v - dev_dash.Sys.cur_zero_v) / dev_dash.Sys.cur_sens;
    if (i_a < 0.0f) i_a = -i_a;
    current_sensor_A = i_a;
  }
}
