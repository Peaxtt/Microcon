/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_BTN_Pin GPIO_PIN_13
#define RESET_BTN_GPIO_Port GPIOC
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define POWER_LATCH_Pin GPIO_PIN_1
#define POWER_LATCH_GPIO_Port GPIOC
#define ESTOP_Pin GPIO_PIN_2
#define ESTOP_GPIO_Port GPIOC
#define MOTOR_DIR_Pin GPIO_PIN_0
#define MOTOR_DIR_GPIO_Port GPIOA
#define REED_UP_Pin GPIO_PIN_1
#define REED_UP_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define REED_DOWN_Pin GPIO_PIN_4
#define REED_DOWN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define REED_GRIP_Pin GPIO_PIN_0
#define REED_GRIP_GPIO_Port GPIOB
#define POWER_BTN_Pin GPIO_PIN_10
#define POWER_BTN_GPIO_Port GPIOB
#define GRIPPER_Pin GPIO_PIN_11
#define GRIPPER_GPIO_Port GPIOB
#define EMER_OUTPUT_Pin GPIO_PIN_14
#define EMER_OUTPUT_GPIO_Port GPIOB
#define PNEUMATIC_Pin GPIO_PIN_6
#define PNEUMATIC_GPIO_Port GPIOC
#define TOWER_G_Pin GPIO_PIN_7
#define TOWER_G_GPIO_Port GPIOC
#define TOWER_R_Pin GPIO_PIN_8
#define TOWER_R_GPIO_Port GPIOC
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define RESET_LED_Pin GPIO_PIN_4
#define RESET_LED_GPIO_Port GPIOB
#define MODE_Pin GPIO_PIN_5
#define MODE_GPIO_Port GPIOB
#define MODE_EXTI_IRQn EXTI9_5_IRQn
#define TOWER_Y_Pin GPIO_PIN_7
#define TOWER_Y_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* =========================================================
 * NEW PERIPHERALS — configure in IOC before uncommenting
 * ========================================================= */

/* --- CAN Bus (FDCAN1) -----------------------------------
 * IOC: Connectivity → FDCAN1 → Activated
 *      PA11 = FDCAN1_RX  (AF9)
 *      PA12 = FDCAN1_TX  (AF9)
 *      Nominal bit-rate: 1 Mbit/s (set Prescaler/Seg in IOC)
 *      Clock source: PCLK1 (170 MHz → prescaler=2, Seg1=42, Seg2=12 → 1M)
 * -------------------------------------------------------- */
#define CAN_NODE_ID       0x001U   /* This board's CAN node ID */

/* --- RP2040 Mini UART (UART4) ---------------------------
 * IOC: Connectivity → UART4 → Asynchronous, 115200 8N1
 *      PB8 = UART4_RX  (AF8)
 *      PB9 = UART4_TX  (AF8)
 * -------------------------------------------------------- */
#define RP2040_BAUD       115200U  /* Match RP2040 firmware baud rate */

/* --- Reed Switches (already in IOC, wiring change needed)
 * IOC change: PA1 / PA4 / PB0 → change GPIO_PULLUP → GPIO_PULLDOWN
 * Wiring: NC contact, COM = board 3V3
 *   Normal state (no magnet): NC closed → 3.3V on GPIO → HIGH
 *   Triggered  (magnet near): NC opens  → PULLDOWN     → LOW
 * Logic: active LOW = position reached
 * -------------------------------------------------------- */
#define REED_ACTIVE  GPIO_PIN_RESET   /* active LOW (NC + PULLDOWN + 3V3 COM) */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
