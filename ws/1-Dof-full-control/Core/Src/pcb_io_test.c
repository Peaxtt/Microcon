/*
 * pcb_io_test.c — 4-phase PCB I/O validation for new pin assignment
 *
 * Phase 1  Output blink   — กด Enter ก่อนบลิงก์แต่ละพิน, วัดด้วย multimeter
 * Phase 2  Input pull     — กด Enter หลังถอด connector แล้ว
 * Phase 3  ADC PA0        — กด Enter เพื่ออ่าน, เสียบ sensor หรือไม่ก็ได้
 * Phase 4  RP2040 alive   — กด Enter หลังเสียบ RP2040 แล้ว
 */

#include "pcb_io_test.h"
#include "joystick.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern ADC_HandleTypeDef hadc2;
extern IWDG_HandleTypeDef hiwdg;

/* ── Pin tables ─────────────────────────────────────────────────────────── */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} PinDef_t;

static const PinDef_t OUTPUT_PINS[] = {
    {GPIOD, GPIO_PIN_2,  "PD2  RESET_LED"  },
    {GPIOC, GPIO_PIN_11, "PC11 EMER_OUT"   },
    {GPIOB, GPIO_PIN_12, "PB12 POWER_LATCH"},
    {GPIOB, GPIO_PIN_2,  "PB2  GRIPPER"    },
    {GPIOB, GPIO_PIN_1,  "PB1  PNEUMATIC"  },
    {GPIOB, GPIO_PIN_15, "PB15 TOWER_G"    },
    {GPIOB, GPIO_PIN_14, "PB14 TOWER_Y"    },
    {GPIOB, GPIO_PIN_13, "PB13 TOWER_R"    },
    {GPIOC, GPIO_PIN_7,  "PC7  MOTOR_DIR"  },
    {GPIOC, GPIO_PIN_9,  "PC9  PWM_test"   },
};
#define OUTPUT_COUNT (sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]))

static const PinDef_t PULLUP_INPUTS[] = {
    {GPIOA, GPIO_PIN_15, "PA15 ESTOP"      },
    {GPIOB, GPIO_PIN_7,  "PB7  POWER_BTN"  },
    {GPIOC, GPIO_PIN_13, "PC13 RESET_BTN"  },
    {GPIOC, GPIO_PIN_2,  "PC2  MODE"       },
    {GPIOC, GPIO_PIN_3,  "PC3  HOME_SENSOR"},
};
#define PULLUP_COUNT (sizeof(PULLUP_INPUTS) / sizeof(PULLUP_INPUTS[0]))

static const PinDef_t PULLDOWN_INPUTS[] = {
    {GPIOB, GPIO_PIN_0, "PB0 REED_GRIP"},
    {GPIOA, GPIO_PIN_4, "PA4 REED_DOWN"},
    {GPIOA, GPIO_PIN_7, "PA7 REED_UP  "},
};
#define PULLDOWN_COUNT (sizeof(PULLDOWN_INPUTS) / sizeof(PULLDOWN_INPUTS[0]))

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static UART_HandleTypeDef *s_uart;

static void uprint(const char *s)
{
    HAL_UART_Transmit(s_uart, (const uint8_t *)s, (uint16_t)strlen(s), 500);
}

static void wait_key(void)
{
    uprint("  >>> กด Enter เพื่อดำเนินการต่อ ...\r\n");
    uint8_t c = 0;
    /* poll 100 ms ต่อรอบ — refresh IWDG ระหว่างรอ */
    while (HAL_UART_Receive(s_uart, &c, 1, 100) != HAL_OK) {
        HAL_IWDG_Refresh(&hiwdg);
    }
    HAL_IWDG_Refresh(&hiwdg);
    /* drain \n หรือ echo ที่ค้างอยู่ */
    HAL_Delay(30);
    HAL_IWDG_Refresh(&hiwdg);
    while (HAL_UART_Receive(s_uart, &c, 1, 10) == HAL_OK) {}
}

/* ── Public ─────────────────────────────────────────────────────────────── */
void pcb_io_test(UART_HandleTypeDef *huart)
{
    s_uart = huart;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    huart->Init.BaudRate   = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits   = UART_STOPBITS_1;
    huart->Init.Parity     = UART_PARITY_NONE;
    huart->Init.Mode       = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);
    HAL_Delay(100);

    uprint("\r\n");
    uprint("╔═══════════════════════════════════════╗\r\n");
    uprint("║       PCB I/O VALIDATOR v1.1          ║\r\n");
    uprint("║  New pin assignment — pre-IOC test    ║\r\n");
    uprint("╚═══════════════════════════════════════╝\r\n");
    uprint("กด Enter เพื่อเริ่ม Phase 1 (connector เสียบหรือไม่ก็ได้)\r\n");
    wait_key();

    char    buf[100];
    uint8_t pass_count = 0;
    uint8_t fail_count = 0;

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 1 — Output blink
     * กด Enter ก่อนแต่ละพิน → บลิงก์ 3× → วาง probe multimeter
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 1] Output blink — วาง probe multimeter ทีละพิน\r\n");
    uprint("  วิธีวัด: multimeter → DC Voltage\r\n");
    uprint("           probe แดง = พินนั้น บน PCB connector\r\n");
    uprint("           probe ดำ  = GND บน PCB\r\n");
    uprint("  expect : สลับ 3.3V ↔ 0V (3 ครั้ง, 400ms ต่อครั้ง)\r\n");

    GPIO_InitTypeDef out_cfg = {0};
    out_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    out_cfg.Pull  = GPIO_NOPULL;
    out_cfg.Speed = GPIO_SPEED_FREQ_LOW;

    for (size_t i = 0; i < OUTPUT_COUNT; i++) {
        snprintf(buf, sizeof(buf), "\r\n  (%u/%u) %s\r\n",
                 (unsigned)(i + 1), (unsigned)OUTPUT_COUNT, OUTPUT_PINS[i].name);
        uprint(buf);
        uprint("  วาง probe ที่พินนี้แล้ว");
        wait_key();
        uprint("  กำลัง blink ...\r\n");

        out_cfg.Pin = OUTPUT_PINS[i].pin;
        HAL_GPIO_Init(OUTPUT_PINS[i].port, &out_cfg);

        for (int p = 0; p < 3; p++) {
            HAL_GPIO_WritePin(OUTPUT_PINS[i].port, OUTPUT_PINS[i].pin, GPIO_PIN_SET);
            HAL_IWDG_Refresh(&hiwdg); HAL_Delay(400);
            HAL_GPIO_WritePin(OUTPUT_PINS[i].port, OUTPUT_PINS[i].pin, GPIO_PIN_RESET);
            HAL_IWDG_Refresh(&hiwdg); HAL_Delay(400);
        }
        uprint("  เสร็จ — บันทึกผลแล้วไปพินต่อไป\r\n");
    }
    uprint("\r\n  Phase 1 เสร็จแล้ว\r\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 2 — Input pull
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 2] Input pull\r\n");
    uprint("  *** ถอด connector ทั้งหมดออกก่อน ***\r\n");
    uprint("  (ยกเว้น ST-Link USB ที่ยังต้องต่ออยู่)\r\n");
    wait_key();

    GPIO_InitTypeDef in_cfg = {0};
    in_cfg.Mode  = GPIO_MODE_INPUT;
    in_cfg.Speed = GPIO_SPEED_FREQ_LOW;

    uprint("  PULLUP (expect HIGH เมื่อไม่มี load):\r\n");
    in_cfg.Pull = GPIO_PULLUP;
    for (size_t i = 0; i < PULLUP_COUNT; i++) {
        in_cfg.Pin = PULLUP_INPUTS[i].pin;
        HAL_GPIO_Init(PULLUP_INPUTS[i].port, &in_cfg);
        HAL_Delay(2);
        GPIO_PinState st = HAL_GPIO_ReadPin(PULLUP_INPUTS[i].port, PULLUP_INPUTS[i].pin);
        int ok = (st == GPIO_PIN_SET);
        if (ok) pass_count++; else fail_count++;
        snprintf(buf, sizeof(buf), "    %-22s = %-4s  %s\r\n",
                 PULLUP_INPUTS[i].name, ok ? "HIGH" : "LOW",
                 ok ? "PASS" : "FAIL (GND short?)");
        uprint(buf);
    }

    uprint("  PULLDOWN (expect LOW เมื่อไม่มี load):\r\n");
    in_cfg.Pull = GPIO_PULLDOWN;
    for (size_t i = 0; i < PULLDOWN_COUNT; i++) {
        in_cfg.Pin = PULLDOWN_INPUTS[i].pin;
        HAL_GPIO_Init(PULLDOWN_INPUTS[i].port, &in_cfg);
        HAL_Delay(2);
        GPIO_PinState st = HAL_GPIO_ReadPin(PULLDOWN_INPUTS[i].port, PULLDOWN_INPUTS[i].pin);
        int ok = (st == GPIO_PIN_RESET);
        if (ok) pass_count++; else fail_count++;
        snprintf(buf, sizeof(buf), "    %-22s = %-4s  %s\r\n",
                 PULLDOWN_INPUTS[i].name, st ? "HIGH" : "LOW",
                 ok ? "PASS" : "FAIL (VCC short?)");
        uprint(buf);
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 3 — ADC PA0
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 3] ADC — PA0 (CURRENT_SENSOR)\r\n");
    uprint("  เสียบ sensor connector ถ้ามี (ไม่มีก็ได้ — ดูแค่ว่า pin ไม่ short)\r\n");
    wait_key();

    GPIO_InitTypeDef adc_gpio = {0};
    adc_gpio.Pin  = GPIO_PIN_0;
    adc_gpio.Mode = GPIO_MODE_ANALOG;
    adc_gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &adc_gpio);
    HAL_Delay(5);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(5);

    static uint32_t local_dma_buf[64];
    HAL_StatusTypeDef dma_st = HAL_ADC_Start_DMA(&hadc2, local_dma_buf, 64);
    snprintf(buf, sizeof(buf), "  DMA_Start=%d  buf[0]=%lu buf[1]=%lu buf[2]=%lu\r\n",
             (int)dma_st,
             (unsigned long)local_dma_buf[0],
             (unsigned long)local_dma_buf[1],
             (unsigned long)local_dma_buf[2]);
    uprint(buf);
    HAL_Delay(200);
    HAL_ADC_Stop_DMA(&hadc2);
    snprintf(buf, sizeof(buf), "  after 200ms: buf[0]=%lu buf[1]=%lu buf[2]=%lu\r\n",
             (unsigned long)local_dma_buf[0],
             (unsigned long)local_dma_buf[1],
             (unsigned long)local_dma_buf[2]);
    uprint(buf);

    /* Direct register read — stop, re-enable ADC, single conversion */
    /* 1. Stop any ongoing conversion */
    if (hadc2.Instance->CR & ADC_CR_ADSTART) {
        SET_BIT(hadc2.Instance->CR, ADC_CR_ADSTP);
        uint32_t tw = 0;
        while ((hadc2.Instance->CR & ADC_CR_ADSTP) && ++tw < 100000U) {}
    }
    /* 2. Disable DMA + CONT so we get a single software-triggered conversion */
    CLEAR_BIT(hadc2.Instance->CFGR, ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_CONT);
    /* 3. Re-enable ADC if HAL_ADC_Stop_DMA disabled it */
    uint32_t ta = 0;
    if (!(hadc2.Instance->CR & ADC_CR_ADEN)) {
        SET_BIT(hadc2.Instance->CR, ADC_CR_ADEN);
        while (!(hadc2.Instance->ISR & ADC_ISR_ADRDY) && ++ta < 200000U) {}
    }
    snprintf(buf, sizeof(buf), "  ADEN_wait=%lu  CR=0x%08lX  SQR1=0x%08lX\r\n",
             ta, (unsigned long)hadc2.Instance->CR,
             (unsigned long)hadc2.Instance->SQR1);
    uprint(buf);
    /* 4. Single conversion */
    hadc2.Instance->ISR = 0xFFFFU;
    SET_BIT(hadc2.Instance->CR, ADC_CR_ADSTART);
    uint32_t t = 0;
    while (!(hadc2.Instance->ISR & ADC_ISR_EOC) && ++t < 200000U) {}
    uint32_t direct_raw = hadc2.Instance->DR;
    snprintf(buf, sizeof(buf), "  direct_raw=%lu  timeout=%lu  ISR=0x%08lX\r\n",
             (unsigned long)direct_raw, (unsigned long)(t >= 200000U),
             (unsigned long)hadc2.Instance->ISR);
    uprint(buf);

    uint32_t sum = 0;
    for (int s = 0; s < 64; s++) sum += local_dma_buf[s];
    uint32_t raw_avg = sum / 64u;
    float    voltage = (float)raw_avg * 3.3f / 4095.0f;

    /* print voltage without %f (newlib-nano ไม่รองรับ float printf) */
    uint32_t v_int  = (uint32_t)voltage;
    uint32_t v_frac = (uint32_t)((voltage - (float)v_int) * 1000.0f);
    snprintf(buf, sizeof(buf), "  PA0 raw=%lu  voltage=%lu.%03lu V\r\n",
             (unsigned long)raw_avg, (unsigned long)v_int, (unsigned long)v_frac);
    uprint(buf);

    if (raw_avg >= 4090u) {
        uprint("  FAIL — stuck HIGH >3.3V (VCC short on PA0?)\r\n");
        fail_count++;
    } else if (raw_avg <= 41u) {
        uprint("  FAIL — stuck LOW (GND short on PA0?)\r\n");
        fail_count++;
    } else if (voltage > 1.3f && voltage < 2.0f) {
        uprint("  PASS — ~1.65V midpoint (sensor 0A)\r\n");
        pass_count++;
    } else {
        uprint("  WARN — readable แต่ไม่อยู่ใน 0A range (sensor อาจไม่ได้เสียบ — OK)\r\n");
        pass_count++;
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 4 — RP2040 alive
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 4] RP2040 alive — PB9 (USART3 RX idle)\r\n");
    uprint("  *** เสียบ connector RP2040 ให้เรียบร้อยก่อน ***\r\n");
    wait_key();

    GPIO_InitTypeDef rp_cfg = {0};
    rp_cfg.Pin   = GPIO_PIN_9;
    rp_cfg.Mode  = GPIO_MODE_INPUT;
    rp_cfg.Pull  = GPIO_PULLDOWN;
    rp_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &rp_cfg);
    HAL_Delay(10);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) {
        uprint("  PB9 = HIGH  →  PASS (RP2040 alive)\r\n");
        pass_count++;
    } else {
        uprint("  PB9 = LOW   →  FAIL\r\n");
        uprint("  เช็ค: RP2040 ได้รับไฟมั้ย? (LED บน RP2040 ติดมั้ย)\r\n");
        uprint("         connector เสียบแน่นครบทุก pin มั้ย?\r\n");
        fail_count++;
    }

    /* ── Summary ─────────────────────────────────────────────────────────── */
    uprint("\r\n");
    uprint("╔═══════════════════════════════════════╗\r\n");
    uprint("║            TEST SUMMARY               ║\r\n");
    snprintf(buf, sizeof(buf),
             "║  PASS: %-2u   FAIL: %-2u                  ║\r\n",
             pass_count, fail_count);
    uprint(buf);
    if (fail_count == 0) {
        uprint("║  ALL PASS — safe to commit IOC        ║\r\n");
    } else {
        uprint("║  FAILURES FOUND — fix before IOC      ║\r\n");
    }
    uprint("╚═══════════════════════════════════════╝\r\n");
    uprint("\r\n");

    while (1) { __NOP(); }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Interactive web tester — command loop (used with docs/pin_tester.html)
 * LPUART1 19200 8E1, line-delimited ASCII commands
 * ═══════════════════════════════════════════════════════════════════════════ */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern volatile float current_sensor_A;

typedef struct { GPIO_TypeDef *port; uint16_t pin; const char *name; } ItPin_t;

static const ItPin_t IT_OUT[] = {
    {GPIOB, GPIO_PIN_15, "TOWER_G"    },
    {GPIOB, GPIO_PIN_14, "TOWER_Y"    },
    {GPIOB, GPIO_PIN_13, "TOWER_R"    },
    {GPIOB, GPIO_PIN_2,  "GRIPPER"    },
    {GPIOB, GPIO_PIN_1,  "PNEUMATIC"  },
    {GPIOB, GPIO_PIN_12, "POWER_LATCH"},
    {GPIOC, GPIO_PIN_11, "EMER_OUT"   },
    {GPIOD, GPIO_PIN_2,  "RESET_LED"  },
    {GPIOC, GPIO_PIN_7,  "MOTOR_DIR"  },
};
#define IT_OUT_N (sizeof(IT_OUT)/sizeof(IT_OUT[0]))

static const ItPin_t IT_IN[] = {
    {GPIOA, GPIO_PIN_15, "ESTOP"      },
    {GPIOB, GPIO_PIN_7,  "POWER_BTN"  },
    {GPIOC, GPIO_PIN_13, "RESET_BTN"  },
    {GPIOC, GPIO_PIN_2,  "MODE"       },
    {GPIOC, GPIO_PIN_3,  "HOME_SENSOR"},
    {GPIOB, GPIO_PIN_0,  "REED_GRIP"  },
    {GPIOA, GPIO_PIN_4,  "REED_DOWN"  },
    {GPIOA, GPIO_PIN_7,  "REED_UP"    },
};
#define IT_IN_N (sizeof(IT_IN)/sizeof(IT_IN[0]))

static void it_tx(UART_HandleTypeDef *h, const char *s)
{
    HAL_UART_Transmit(h, (const uint8_t *)s, (uint16_t)strlen(s), 200);
}

static void it_set_pwm(uint32_t pct)
{
    if (pct > 100u) pct = 100u;
    uint32_t ccr = pct * htim3.Init.Period / 100u;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}

static uint8_t it_handle(UART_HandleTypeDef *h, char *cmd)
{
    char buf[320];

    /* PING */
    if (strcmp(cmd, "PING") == 0) { it_tx(h, "PONG\r\n"); return 0; }

    /* EXIT */
    if (strcmp(cmd, "EXIT") == 0) { it_tx(h, "BYE\r\n"); return 1; }

    /* SCAN — all inputs + joystick state */
    if (strcmp(cmd, "SCAN") == 0) {
        strcpy(buf, "SCAN:");
        for (uint8_t i = 0; i < IT_IN_N; i++) {
            char tmp[24];
            int v = (HAL_GPIO_ReadPin(IT_IN[i].port, IT_IN[i].pin) == GPIO_PIN_SET) ? 1 : 0;
            snprintf(tmp, sizeof(tmp), "%s=%d%s", IT_IN[i].name, v, i < IT_IN_N-1 ? "," : "");
            strncat(buf, tmp, sizeof(buf)-strlen(buf)-1);
        }
        /* Append encoder + ADC */
        {
            int32_t enc = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
            uint32_t adc_ma = (uint32_t)(current_sensor_A * 1000.0f);
            char enc_buf[48];
            snprintf(enc_buf, sizeof(enc_buf), "|ENC:cnt=%ld,deg=%.1f|ADC:mA=%lu",
                     (long)enc, (float)enc * (360.0f / 8192.0f), (unsigned long)adc_ma);
            strncat(buf, enc_buf, sizeof(buf)-strlen(buf)-1);
        }
        /* Append joystick data after pipe separator */
        char joy_buf[96];
        snprintf(joy_buf, sizeof(joy_buf),
            "|JOY:rp=%d,joy=%d,lx=%d,ly=%d,rx=%d,ry=%d,lt=%d,rt=%d,btns=%04X\r\n",
            (int)joy_rp2040_alive(), (int)joy_is_connected(),
            (int)joy_lx(), (int)joy_ly(), (int)joy_rx(), (int)joy_ry(),
            (int)joy_lt(), (int)joy_rt(), (unsigned)joy_raw_buttons());
        strncat(buf, joy_buf, sizeof(buf)-strlen(buf)-1);
        it_tx(h, buf);
        return 0;
    }

    /* SET:NAME:0/1 — set output or PWM duty */
    if (strncmp(cmd, "SET:", 4) == 0) {
        char *name  = cmd + 4;
        char *colon = strchr(name, ':');
        if (!colon) { it_tx(h, "ERR:BAD_FORMAT\r\n"); return 0; }
        *colon = '\0';
        int val = (int)strtol(colon + 1, NULL, 10);

        if (strcmp(name, "PWM") == 0) {
            it_set_pwm((uint32_t)(val < 0 ? 0 : val));
            it_tx(h, "OK\r\n"); return 0;
        }
        for (uint8_t i = 0; i < IT_OUT_N; i++) {
            if (strcmp(IT_OUT[i].name, name) == 0) {
                HAL_GPIO_WritePin(IT_OUT[i].port, IT_OUT[i].pin,
                                  val ? GPIO_PIN_SET : GPIO_PIN_RESET);
                it_tx(h, "OK\r\n"); return 0;
            }
        }
        it_tx(h, "ERR:UNKNOWN_PIN\r\n"); return 0;
    }

    /* TOG:NAME — toggle output, reply with new state */
    if (strncmp(cmd, "TOG:", 4) == 0) {
        char *name = cmd + 4;
        for (uint8_t i = 0; i < IT_OUT_N; i++) {
            if (strcmp(IT_OUT[i].name, name) == 0) {
                HAL_GPIO_TogglePin(IT_OUT[i].port, IT_OUT[i].pin);
                int v = (HAL_GPIO_ReadPin(IT_OUT[i].port, IT_OUT[i].pin) == GPIO_PIN_SET) ? 1 : 0;
                snprintf(buf, sizeof(buf), "VAL:%s=%d\r\n", name, v);
                it_tx(h, buf); return 0;
            }
        }
        it_tx(h, "ERR:UNKNOWN_PIN\r\n"); return 0;
    }

    /* JOY — joystick only (short response for fast polling) */
    if (strcmp(cmd, "JOY") == 0) {
        char joy_buf[64];
        snprintf(joy_buf, sizeof(joy_buf),
            "JOY:rp=%d,joy=%d,lx=%d,ly=%d,rx=%d,ry=%d,lt=%d,rt=%d,btns=%04X\r\n",
            (int)joy_rp2040_alive(), (int)joy_is_connected(),
            (int)joy_lx(), (int)joy_ly(), (int)joy_rx(), (int)joy_ry(),
            (int)joy_lt(), (int)joy_rt(), (unsigned)joy_raw_buttons());
        it_tx(h, joy_buf);
        return 0;
    }

    /* J — compact joystick (~49 chars max @ extreme values, ~28ms TX @ 19200 8E1) */
    if (strcmp(cmd, "J") == 0) {
        char jb[56];
        snprintf(jb, sizeof(jb), "J:%d,%d,%d,%d,%d,%d,%d,%d,%04X\r\n",
            (int)joy_rp2040_alive(), (int)joy_is_connected(),
            (int)joy_lx(), (int)joy_ly(), (int)joy_rx(), (int)joy_ry(),
            (int)joy_lt(), (int)joy_rt(), (unsigned)joy_raw_buttons());
        it_tx(h, jb);
        return 0;
    }

    it_tx(h, "ERR:UNKNOWN_CMD\r\n");
    return 0;
}

void pcb_interactive_test(UART_HandleTypeDef *huart)
{
    /* Safe state — all outputs off, PWM = 0 */
    for (uint8_t i = 0; i < IT_OUT_N; i++)
        HAL_GPIO_WritePin(IT_OUT[i].port, IT_OUT[i].pin, GPIO_PIN_RESET);
    it_set_pwm(0);

    it_tx(huart, "READY:PCB_TESTER_v1\r\n");

    uint8_t rx_byte, cmd_buf[64], cmd_idx = 0;

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        if (HAL_UART_Receive(huart, &rx_byte, 1, 20) == HAL_OK) {
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (cmd_idx > 0) {
                    cmd_buf[cmd_idx] = '\0';
                    cmd_idx = 0;
                    if (it_handle(huart, (char *)cmd_buf)) break;
                }
            } else if (cmd_idx < 63) {
                cmd_buf[cmd_idx++] = rx_byte;
            }
        }
    }

    /* Restore safe state on exit */
    for (uint8_t i = 0; i < IT_OUT_N; i++)
        HAL_GPIO_WritePin(IT_OUT[i].port, IT_OUT[i].pin, GPIO_PIN_RESET);
    it_set_pwm(0);
}
