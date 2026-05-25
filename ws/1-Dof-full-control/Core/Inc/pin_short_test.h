#ifndef PIN_SHORT_TEST_H
#define PIN_SHORT_TEST_H

#include "main.h"

// Call once at startup (before while(1))
// Open terminal at 115200 8N1 (ST-Link VCP)
// Run 1: NUCLEO standalone → save output
// Run 2: NUCLEO on PCB    → save output
// Lines that appear ONLY in Run 2 = PCB causing the problem
void pin_short_test(UART_HandleTypeDef *huart);

#endif
