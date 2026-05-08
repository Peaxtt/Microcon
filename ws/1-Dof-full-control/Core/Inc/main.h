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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB

/* 1-DOF Robot Control Pin Mappings */

/* 1. Motor Control */
#define MOTOR_PWM_Pin GPIO_PIN_6
#define MOTOR_PWM_GPIO_Port GPIOA
#define MOTOR_DIR_Pin GPIO_PIN_1
#define MOTOR_DIR_GPIO_Port GPIOA

/* 2. Encoder */
#define ENCODER_B_Pin GPIO_PIN_8
#define ENCODER_B_GPIO_Port GPIOA
#define ENCODER_A_Pin GPIO_PIN_9
#define ENCODER_A_GPIO_Port GPIOA

/* 3. Sensors & Buttons */
#define ESTOP_Pin GPIO_PIN_13
#define ESTOP_GPIO_Port GPIOB
#define HOME_Pin GPIO_PIN_5
#define HOME_GPIO_Port GPIOB
#define RESET_BTN_Pin GPIO_PIN_1
#define RESET_BTN_GPIO_Port GPIOB
#define POWER_BTN_Pin GPIO_PIN_10
#define POWER_BTN_GPIO_Port GPIOB
#define MODE_BTN_Pin GPIO_PIN_0
#define MODE_BTN_GPIO_Port GPIOC
#define REED_UP_Pin GPIO_PIN_0
#define REED_UP_GPIO_Port GPIOA
#define REED_DOWN_Pin GPIO_PIN_4
#define REED_DOWN_GPIO_Port GPIOA
#define REED_GRIP_Pin GPIO_PIN_0
#define REED_GRIP_GPIO_Port GPIOB

/* 4. Relay Control */
#define TOWER_G_Pin GPIO_PIN_7
#define TOWER_G_GPIO_Port GPIOC
#define TOWER_Y_Pin GPIO_PIN_7
#define TOWER_Y_GPIO_Port GPIOB
#define TOWER_R_Pin GPIO_PIN_8
#define TOWER_R_GPIO_Port GPIOC
#define PNEUMATIC_Pin GPIO_PIN_1
#define PNEUMATIC_GPIO_Port GPIOC
#define GRIPPER_Pin GPIO_PIN_11
#define GRIPPER_GPIO_Port GPIOB
#define RESET_LED_Pin GPIO_PIN_4
#define RESET_LED_GPIO_Port GPIOB
#define POWER_LATCH_Pin GPIO_PIN_14
#define POWER_LATCH_GPIO_Port GPIOB

/* 5. Analog Sensors */
#define CURRENT_SENSE_Pin GPIO_PIN_5
#define CURRENT_SENSE_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
