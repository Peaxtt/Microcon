/*
 * pin_short_test.c
 *
 * Run once at startup (before while(1)) to detect PCB pin shorts.
 * LPUART1 is temporarily reinitialised to 115200 8N1 for terminal output.
 * Open serial terminal at 115200 8N1 (ST-Link VCP).
 *
 * Usage:
 *   Run 1 – NUCLEO standalone  → capture output → "baseline.txt"
 *   Run 2 – NUCLEO on PCB      → capture output → "pcb.txt"
 *   Lines only in pcb.txt = PCB is causing the problem.
 */

#include "pin_short_test.h"
#include <stdio.h>
#include <string.h>

/* ── Pin table ──────────────────────────────────────────────────────────────
 * Excluded from testing:
 *   PA2 / PA3  – LPUART1 TX/RX  (used for output here)
 *   PA13/ PA14 – SWDIO / SWCLK  (would kill debug connection)
 *   PC14/ PC15 – OSC32_IN/OUT   (LSE crystal)
 * -------------------------------------------------------------------------- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} PinEntry_t;

static const PinEntry_t s_pins[] = {
    /* GPIOA — skip PA1(VCC) PA2/PA3(UART) PA11/PA12(USB-VCC) PA13/PA14(SWD) */
    {GPIOA, GPIO_PIN_0,  "PA0" },
    {GPIOA, GPIO_PIN_4,  "PA4" }, {GPIOA, GPIO_PIN_5,  "PA5" },
    {GPIOA, GPIO_PIN_6,  "PA6" }, {GPIOA, GPIO_PIN_7,  "PA7" },
    {GPIOA, GPIO_PIN_8,  "PA8" }, {GPIOA, GPIO_PIN_9,  "PA9" },
    {GPIOA, GPIO_PIN_10, "PA10"}, {GPIOA, GPIO_PIN_15, "PA15"},

    /* GPIOB — skip PB11(GND_SHORT) */
    {GPIOB, GPIO_PIN_0,  "PB0" }, {GPIOB, GPIO_PIN_1,  "PB1" },
    {GPIOB, GPIO_PIN_2,  "PB2" }, {GPIOB, GPIO_PIN_3,  "PB3" },
    {GPIOB, GPIO_PIN_4,  "PB4" }, {GPIOB, GPIO_PIN_5,  "PB5" },
    {GPIOB, GPIO_PIN_6,  "PB6" }, {GPIOB, GPIO_PIN_7,  "PB7" },
    {GPIOB, GPIO_PIN_8,  "PB8" }, {GPIOB, GPIO_PIN_9,  "PB9" },
    {GPIOB, GPIO_PIN_10, "PB10"},
    {GPIOB, GPIO_PIN_12, "PB12"}, {GPIOB, GPIO_PIN_13, "PB13"},
    {GPIOB, GPIO_PIN_14, "PB14"}, {GPIOB, GPIO_PIN_15, "PB15"},

    /* GPIOC — skip PC0/PC1(short) PC4/PC5(VCC) PC10/PC12(short) PC14/PC15(LSE) */
    {GPIOC, GPIO_PIN_2,  "PC2" }, {GPIOC, GPIO_PIN_3,  "PC3" },
    {GPIOC, GPIO_PIN_6,  "PC6" }, {GPIOC, GPIO_PIN_7,  "PC7" },
    {GPIOC, GPIO_PIN_8,  "PC8" }, {GPIOC, GPIO_PIN_9,  "PC9" },
    {GPIOC, GPIO_PIN_11, "PC11"}, {GPIOC, GPIO_PIN_13, "PC13"},

    /* GPIOD */
    {GPIOD, GPIO_PIN_0,  "PD0" }, {GPIOD, GPIO_PIN_1,  "PD1" },
    {GPIOD, GPIO_PIN_2,  "PD2" },
};

#define PIN_COUNT  (sizeof(s_pins) / sizeof(s_pins[0]))

/* ── Internal helpers ───────────────────────────────────────────────────── */
static UART_HandleTypeDef *s_uart;

static void uprint(const char *s)
{
    HAL_UART_Transmit(s_uart, (const uint8_t *)s, (uint16_t)strlen(s), 300);
}

static void set_all_input(uint32_t pull)
{
    GPIO_InitTypeDef cfg = {0};
    cfg.Mode  = GPIO_MODE_INPUT;
    cfg.Pull  = pull;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    for (size_t i = 0; i < PIN_COUNT; i++) {
        cfg.Pin = s_pins[i].pin;
        HAL_GPIO_Init(s_pins[i].port, &cfg);
    }
}

static void set_input_one(size_t idx, uint32_t pull)
{
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin   = s_pins[idx].pin;
    cfg.Mode  = GPIO_MODE_INPUT;
    cfg.Pull  = pull;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(s_pins[idx].port, &cfg);
}

/* ── Public function ────────────────────────────────────────────────────── */
void pin_short_test(UART_HandleTypeDef *huart)
{
    s_uart = huart;

    /* Ensure GPIO clocks are on */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ── Reinit LPUART1 → 115200 8N1 for human-readable terminal output ── */
    huart->Init.BaudRate   = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits   = UART_STOPBITS_1;
    huart->Init.Parity     = UART_PARITY_NONE;
    huart->Init.Mode       = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);
    HAL_Delay(100);

    /* ── Header ─────────────────────────────────────────────────────────── */
    uprint("\r\n");
    uprint("╔═══════════════════════════════════════╗\r\n");
    uprint("║         PIN SHORT FINDER v1.0         ║\r\n");
    uprint("╚═══════════════════════════════════════╝\r\n");
    uprint("Skipped: PA2 PA3(LPUART1)  PA13 PA14(SWD)  PC14 PC15(OSC32)\r\n");

    char    buf[96];
    uint8_t vcc_short[PIN_COUNT];
    uint8_t gnd_short[PIN_COUNT];
    memset(vcc_short, 0, sizeof(vcc_short));
    memset(gnd_short, 0, sizeof(gnd_short));

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 1 – VCC short
     * All pins → INPUT_PULLDOWN.  Any pin that reads HIGH is driven by VCC
     * through the PCB (short to 3.3 V or 5 V rail).
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 1] VCC short — all PULLDOWN, read HIGH\r\n");
    set_all_input(GPIO_PULLDOWN);
    HAL_Delay(5);

    uint8_t found = 0;
    for (size_t i = 0; i < PIN_COUNT; i++) {
        if (HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_SET) {
            vcc_short[i] = 1;
            snprintf(buf, sizeof(buf), "  VCC_SHORT  %s\r\n", s_pins[i].name);
            uprint(buf);
            found = 1;
        }
    }
    if (!found) uprint("  (none)\r\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 2 – GND short
     * All pins → INPUT_PULLUP.  Any pin that reads LOW is sunk by GND
     * through the PCB.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 2] GND short — all PULLUP, read LOW\r\n");
    set_all_input(GPIO_PULLUP);
    HAL_Delay(5);

    found = 0;
    for (size_t i = 0; i < PIN_COUNT; i++) {
        if (HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_RESET) {
            gnd_short[i] = 1;
            snprintf(buf, sizeof(buf), "  GND_SHORT  %s\r\n", s_pins[i].name);
            uprint(buf);
            found = 1;
        }
    }
    if (!found) uprint("  (none)\r\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Phase 3 – Pin-to-pin shorts
     * For each driver pin:
     *   1. Set all pins INPUT_PULLDOWN.
     *   2. Drive driver pin OUTPUT HIGH.
     *   3. Wait 2 ms for capacitance to settle.
     *   4. Read every other pin — any HIGH means that pin is shorted to driver.
     *      (Skip pins already known to be VCC-shorted: they always read HIGH.)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uprint("\r\n[Phase 3] Pin-to-pin — drive HIGH, others PULLDOWN\r\n");
    uprint("  (VCC-shorted pins excluded from reader list)\r\n");

    GPIO_InitTypeDef out_cfg = {0};
    out_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    out_cfg.Pull  = GPIO_NOPULL;
    out_cfg.Speed = GPIO_SPEED_FREQ_LOW;

    uint8_t any_found = 0;

    for (size_t drv = 0; drv < PIN_COUNT; drv++) {
        /* All inputs with pull-down */
        set_all_input(GPIO_PULLDOWN);

        /* Drive pin HIGH */
        out_cfg.Pin = s_pins[drv].pin;
        HAL_GPIO_Init(s_pins[drv].port, &out_cfg);
        HAL_GPIO_WritePin(s_pins[drv].port, s_pins[drv].pin, GPIO_PIN_SET);
        HAL_Delay(2);

        uint8_t header_printed = 0;
        for (size_t rd = 0; rd < PIN_COUNT; rd++) {
            if (rd == drv)        continue;  /* self */
            if (vcc_short[rd])    continue;  /* always HIGH – not useful */
            if (HAL_GPIO_ReadPin(s_pins[rd].port, s_pins[rd].pin) == GPIO_PIN_SET) {
                if (!header_printed) {
                    snprintf(buf, sizeof(buf), "  %-5s shorted to:", s_pins[drv].name);
                    uprint(buf);
                    header_printed = 1;
                    any_found = 1;
                }
                snprintf(buf, sizeof(buf), " %s", s_pins[rd].name);
                uprint(buf);
            }
        }
        if (header_printed) uprint("\r\n");

        /* Release driver pin back to input */
        set_input_one(drv, GPIO_PULLDOWN);
    }
    if (!any_found) uprint("  (none)\r\n");

    /* ── Footer ─────────────────────────────────────────────────────────── */
    uprint("\r\n");
    uprint("╔═══════════════════════════════════════╗\r\n");
    uprint("║           TEST COMPLETE               ║\r\n");
    uprint("║  Capture output, then reflash normal  ║\r\n");
    uprint("╚═══════════════════════════════════════╝\r\n");
    uprint("\r\n");

    /* Block here so the output is fully visible before anything else runs */
    while (1) { __NOP(); }
}
