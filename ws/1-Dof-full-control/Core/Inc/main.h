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
#define MODE_Pin GPIO_PIN_2
#define MODE_GPIO_Port GPIOC
#define HOME_SENSOR_Pin GPIO_PIN_3
#define HOME_SENSOR_GPIO_Port GPIOC
#define CURRENT_ADC_Pin GPIO_PIN_0
#define CURRENT_ADC_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define REED_DOWN_Pin GPIO_PIN_4
#define REED_DOWN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define REED_UP_Pin GPIO_PIN_7
#define REED_UP_GPIO_Port GPIOA
#define REED_GRIP_Pin GPIO_PIN_0
#define REED_GRIP_GPIO_Port GPIOB
#define PNEUMATIC_Pin GPIO_PIN_1
#define PNEUMATIC_GPIO_Port GPIOB
#define GRIPPER_Pin GPIO_PIN_2
#define GRIPPER_GPIO_Port GPIOB
#define POWER_LATCH_Pin GPIO_PIN_12
#define POWER_LATCH_GPIO_Port GPIOB
#define TOWER_R_Pin GPIO_PIN_13
#define TOWER_R_GPIO_Port GPIOB
#define TOWER_Y_Pin GPIO_PIN_14
#define TOWER_Y_GPIO_Port GPIOB
#define TOWER_G_Pin GPIO_PIN_15
#define TOWER_G_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_7
#define MOTOR_DIR_GPIO_Port GPIOC
#define PWM_OUT_Pin GPIO_PIN_9
#define PWM_OUT_GPIO_Port GPIOC
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define ESTOP_Pin GPIO_PIN_15
#define ESTOP_GPIO_Port GPIOA
#define EMER_OUTPUT_Pin GPIO_PIN_11
#define EMER_OUTPUT_GPIO_Port GPIOC
#define RESET_LED_Pin GPIO_PIN_2
#define RESET_LED_GPIO_Port GPIOD
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define POWER_BTN_Pin GPIO_PIN_7
#define POWER_BTN_GPIO_Port GPIOB
#define JOY_RX_Pin GPIO_PIN_8
#define JOY_RX_GPIO_Port GPIOB
#define JOY_TX_Pin GPIO_PIN_9
#define JOY_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* =========================================================
 * NEW PERIPHERALS — configure in IOC before uncommenting
 * ========================================================= */

/* --- CAN Bus (FDCAN1) — configured in IOC, handle = hfdcan1 -------------
 *      PA11 = FDCAN1_RX (AF9),  PA12 = FDCAN1_TX (AF9)
 *      Nominal: Prescaler=5, Seg1=25, Seg2=8, SJW=2 → 1 Mbit/s @ 170MHz
 *      Mode: Classic CAN (FDCAN_FRAME_CLASSIC), FDCAN_MODE_NORMAL
 *      Global filter: accept-all → RX FIFO0
 *      External transceiver: TJA1050 (TX→TXD, RX←RXD)
 * -------------------------------------------------------- */
#define CAN_NODE_ID       0x001U   /* This board's CAN node ID */

/* --- RP2040 Mini — already connected via USART3 (PC10/PC11) @ 460800 8N1
 * No additional UART needed.
 * -------------------------------------------------------- */

/* --- Reed Switches (already in IOC, wiring change needed)
 * IOC change: PA1 / PA4 / PB0 → change GPIO_PULLUP → GPIO_PULLDOWN
 * Wiring: NC contact, COM = board 3V3
 *   Normal state (no magnet): NC closed → 3.3V on GPIO → HIGH
 *   Triggered  (magnet near): NC opens  → PULLDOWN     → LOW
 * Logic: active LOW = position reached
 * -------------------------------------------------------- */
#define REED_ACTIVE  GPIO_PIN_RESET   /* active LOW (NC + PULLDOWN + 3V3 COM) */

/* --- Home Sensor (proximity PNP → opto-isolator → GPIO) ----------------
 * Pin: PC3 — EXTI rising edge, PULLUP
 * Normal (no blade): NPN sinks → LOW (0) = not triggered
 * Triggered (blade): NPN off → PULLUP → HIGH (1) = triggered
 * -------------------------------------------------------- */
#define HOME_SENSOR_Pin       GPIO_PIN_3
#define HOME_SENSOR_GPIO_Port GPIOC
#define HOME_SENSOR_ACTIVE    GPIO_PIN_SET   /* active HIGH — rising edge */

/* --- Homing parameters ------------------------------------------------- */
// HOMING_FAST_SPEED moved to main.c as volatile float for Live Expressions tuning
// HOMING_SLOW_SPEED moved to main.c as volatile float for Live Expressions tuning
#define HOMING_BACKOFF_DEG  20.0f   /* degrees to back off after first hit  */
#define MAX_ANGLE_DEG       540.0f  /* +1.5 rev soft limit                  */
#define MIN_ANGLE_DEG      -540.0f  /* -1.5 rev soft limit                  */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
