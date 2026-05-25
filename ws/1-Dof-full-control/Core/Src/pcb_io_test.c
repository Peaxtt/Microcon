/*
 * pcb_io_test.c — 4-phase PCB I/O validation for new pin assignment
 *
 * Phase 1  Output blink   — blink every output 3× (400 ms on/off), measure with multimeter
 * Phase 2  Input pull     — PULLUP inputs expect HIGH, PULLDOWN inputs expect LOW when floating
 * Phase 3  ADC PA0        — reconfigure PA0 analog, read 64 samples, report voltage
 * Phase 4  RP2040 alive   — PB9 INPUT_PULLDOWN: HIGH = RP2040 powered & USART3 idle
 *
 * Usage:
 *   pcb_io_test(&hlpuart1);  at very first line of USER CODE BEGIN 2
 *   Open terminal 115200 8N1
 *   Function blocks at the end — reflash to restore normal firmware.
 */

#include "pcb_io_test.h"
#include <stdio.h>
#include <string.h>

extern ADC_HandleTypeDef hadc2;

/* ── Pin tables ─────────────────────────────────────────────────────────── */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} PinDef_t;

static const PinDef_t OUTPUT_PINS[] = {
    {GPIOD, GPIO_PIN_2,  "PD2  RESET_LED"  },
    {GPIOC, GPIO_PIN_6,  "PC6  EMER_OUT"   },
    {GPIOB, GPIO_PIN_12, "PB12 POWER_LATCH"},
    {GPIOB, GPIO_PIN_2,  "PB2  GRIPPER"    },
    {GPIOB, GPIO_PIN_1,  "PB1  PNEUMATIC"  },
    {GPIOB, GPIO_PIN_15, "PB15 TOWER_G"    },
    {GPIOB, GPIO_PIN_14, "PB14 TOWER_Y"    },
    {GPIOB, GPIO_PIN_13, "PB13 TOWER_R"    },
    {GPIOC, GPIO_PIN_8,  "PC8  MOTOR_DIR"  },
    {GPIOC, GPIO_PIN_9,  "PC9  PWM_test"   },
};
#define OUTPUT_COUNT (sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]))

/* PULLUP inputs — expect HIGH when floating (no external load) */
static const PinDef_t PULLUP_INPUTS[] = {
    {GPIOA, GPIO_PIN_15, "PA15 ESTOP"      },
    {GPIOB, GPIO_PIN_7,  "PB7  POWER_BTN"  },
    {GPIOC, GPIO_PIN_13, "PC13 RESET_BTN"  },
    {GPIOC, GPIO_PIN_2,  "PC2  MODE"       },
    {GPIOC, GPIO_PIN_3,  "PC3  HOME_SENSOR"},
};
#define PULLUP_COUNT (sizeof(PULLUP_INPUTS) / sizeof(PULLUP_INPUTS[0]))

/* PULLDOWN inputs — expect LOW when floating */
static const PinDef_t PULLDOWN_INPUTS[] = {
    {GPIOB, GPIO_PIN_0, "PB0 REED_GRIP"},
    {GPIOA, GPIO_PIN_4, "PA4 REED_DOWN"},
    {GPIOA, GPIO_PIN_1, "PA1 REED_UP  "},
};
#define PULLDOWN_COUNT (sizeof(PULLDOWN_INPUTS) / sizeof(PULLDOWN_INPUTS[0]))

/* ── UART helper ─────────────────────────────────────────────────────────── */
static UART_HandleTypeDef *s_uart;

static void uprint(const char *s)
{
    HAL_UART_Transmit(s_uart, (const uint8_t *)s, (uint16_t)strlen(s), 500);
}

/* ── Public ─────────────────────────────────────────────────────────────── */
void pcb_io_test(UART_HandleTypeDef *huart)
{
    s_uart = huart;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Reinit LPUART1 → 115200 8N1 for human-readable terminal */
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
    uprint("║       PCB I/O VALIDATOR v1.0          ║\r\n");
    uprint("║  New pin assignment — pre-IOC test    ║\r\n");
    uprint("╚═══════════════════════════════════════╝\r\n");
    uprint("IMPORTANT: unplug all connectors before Phase 2\r\n");

    char    buf[100];
    uint8_t pass_count = 0;
    uint8_t fail_count = 0;

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 1 — Output blink
     * Drive each output PP, blink 3× (400 ms ON / 400 ms OFF).
     * Probe with multimeter: expect 3.3 V HIGH, 0 V LOW.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 1] Output blink — probe each pin with multimeter\r\n");

    GPIO_InitTypeDef out_cfg = {0};
    out_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    out_cfg.Pull  = GPIO_NOPULL;
    out_cfg.Speed = GPIO_SPEED_FREQ_LOW;

    for (size_t i = 0; i < OUTPUT_COUNT; i++) {
        snprintf(buf, sizeof(buf), "  Blinking %-20s (3 pulses) ...\r\n",
                 OUTPUT_PINS[i].name);
        uprint(buf);

        out_cfg.Pin = OUTPUT_PINS[i].pin;
        HAL_GPIO_Init(OUTPUT_PINS[i].port, &out_cfg);

        for (int p = 0; p < 3; p++) {
            HAL_GPIO_WritePin(OUTPUT_PINS[i].port, OUTPUT_PINS[i].pin, GPIO_PIN_SET);
            HAL_Delay(400);
            HAL_GPIO_WritePin(OUTPUT_PINS[i].port, OUTPUT_PINS[i].pin, GPIO_PIN_RESET);
            HAL_Delay(400);
        }
    }
    uprint("  Phase 1 done.\r\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 2 — Input pull (connectors unplugged)
     * PULLUP  → expect HIGH when floating.  LOW  = GND short on PCB.
     * PULLDOWN → expect LOW  when floating.  HIGH = VCC short on PCB.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 2] Input pull — UNPLUG all connectors now\r\n");
    HAL_Delay(3000);   /* 3 s window to unplug before reading */

    GPIO_InitTypeDef in_cfg = {0};
    in_cfg.Mode  = GPIO_MODE_INPUT;
    in_cfg.Speed = GPIO_SPEED_FREQ_LOW;

    uprint("  PULLUP inputs (expect HIGH):\r\n");
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

    uprint("  PULLDOWN inputs (expect LOW):\r\n");
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
     * Phase 3 — ADC on PA0 (ADC2_IN1 = CURRENT_SENSOR)
     * 64 samples averaged.  Stuck >3.0V = VCC short.  Stuck <0.1V = GND short.
     * With sensor powered and 0A: expect ~1.65V.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 3] ADC — PA0 (ADC2_IN1 CURRENT_SENSOR)\r\n");

    GPIO_InitTypeDef adc_gpio = {0};
    adc_gpio.Pin  = GPIO_PIN_0;
    adc_gpio.Mode = GPIO_MODE_ANALOG;
    adc_gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &adc_gpio);
    HAL_Delay(5);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_1;          /* PA0 = ADC2_IN1 */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(5);

    uint32_t sum = 0;
    for (int s = 0; s < 64; s++) {
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 10);
        sum += HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);
    }
    uint32_t raw_avg = sum / 64u;
    float    voltage = (float)raw_avg * 3.3f / 4095.0f;

    snprintf(buf, sizeof(buf), "  PA0 raw=%lu  voltage=%.3f V\r\n",
             (unsigned long)raw_avg, (double)voltage);
    uprint(buf);

    if (voltage > 3.0f) {
        uprint("  FAIL — stuck HIGH (VCC short on PA0?)\r\n");
        fail_count++;
    } else if (voltage < 0.10f) {
        uprint("  FAIL — stuck LOW (GND short on PA0?)\r\n");
        fail_count++;
    } else if (voltage > 1.3f && voltage < 2.0f) {
        uprint("  PASS — ~1.65V midpoint (sensor at 0A, correct)\r\n");
        pass_count++;
    } else {
        uprint("  WARN — voltage readable but outside 0A range (sensor unplugged?)\r\n");
        pass_count++;   /* readable = not shorted, good enough pre-IOC */
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 4 — RP2040 alive via PB9 (USART3 RX)
     * UART idle line is held HIGH by the transmitter.
     * Configure PB9 INPUT_PULLDOWN → HIGH = RP2040 powered.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 4] RP2040 alive — PB9 (USART3 RX idle)\r\n");
    uprint("  Plug RP2040 in now if not already connected.\r\n");
    HAL_Delay(2000);

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
        uprint("  PB9 = LOW   →  FAIL (RP2040 off or not connected)\r\n");
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

    /* Block — reflash to exit */
    while (1) { __NOP(); }
}
