/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <arm_math.h>
#include <string.h> // ต้องใช้สำหรับฟังก์ชันรีเซ็ตค่า (memset)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float32_t v;
	float32_t v1;
	float32_t v2;
	float32_t w1;
	float32_t w2;
	float32_t currentOmega;
} MotorState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*motor parameter */
float32_t L = 0.003;
float32_t R = 3.58;
float32_t j = 1.95/100000.0 ;
float32_t b = 6.53/100000.0 ;
float32_t Ke = 0.0425;
float32_t Km = 0.0438;
/*Discrete constant */
float32_t A =0.0;
float32_t B = 0.0;
float32_t C = 0.0;

// SINE SET UP
float32_t sineFreq = 1.0f;
float32_t amplitude = 12.0f;
volatile float32_t vin_Sine = 0.0f;

// RAMP SET UP
float32_t slope = 1.0f;
float32_t ramp_start_time = 1.0f;
volatile float32_t vin_Ramp = 0.0f;

// ตัวแปร 6 เส้น
volatile float32_t fwdOmega_Sine=0.0;
volatile float32_t bwdOmega_Sine=0.0;
volatile float32_t tusOmega_Sine=0.0;
volatile float32_t fwdOmega_Ramp=0.0;
volatile float32_t bwdOmega_Ramp=0.0;
volatile float32_t tusOmega_Ramp=0.0;

MotorState_t stFwdSin = {0}, stBwdSin = {0}, stTusSin = {0};
MotorState_t stFwdRmp = {0}, stBwdRmp = {0}, stTusRmp = {0};

/* --- 🌟 ใช้ความถี่เดียวตามปกติของการทำ HIL Simulation --- */
float32_t freq = 1000.0f;
float32_t TsInS = 0.0f;
volatile float32_t t_plot = 0.0f;

// ==========================================================
// 🌟 สวิตช์ควบคุมการทำงาน (0 = ปิด, 1 = เปิด)
// นำตัวแปรเหล่านี้ไปดึงลง CubeMonitor เพื่อกดเปิด/ปิดแบบ Real-time ได้เลย
// ==========================================================
volatile uint8_t enable_fwd = 1; // 1 = เปิดรัน Forward
volatile uint8_t enable_bwd = 1; // 1 = เปิดรัน Backward
volatile uint8_t enable_tus = 1; // 1 = เปิดรัน Tustin
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
float32_t sineGenerator(float32_t t_sec);
float32_t rampGenerator(float32_t t_sec);
void forwardEuler(float32_t Vin, MotorState_t *s);
void backwardEuler(float32_t Vin, MotorState_t *s);
void tustin(float32_t Vin, MotorState_t *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// แก้ไขการหารให้เป็นทศนิยม เพื่อป้องกันผลลัพธ์เป็น 0
		A = L * j;
		B = (L * b) + (j * R);
		C = (b * R) + (Ke * Km);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();
		SystemClock_Config();
		MX_GPIO_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    TsInS = 1.0f / freq; // freq ตั้งไว้ 1000.0f
    uint32_t ticks_per_loop = HAL_RCC_GetSysClockFreq() / (uint32_t)freq;
    uint32_t last_cycle = DWT->CYCCNT;

    float32_t signal_time = 0.0f;
    t_plot = 0.0f; // เริ่มต้นเวลา plot ที่ 0
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t current_cycle = DWT->CYCCNT;

	  	  	  if ((current_cycle - last_cycle) >= ticks_per_loop)
	  	  	  {
	  	  		  last_cycle += ticks_per_loop;

	  	  		  // นับเวลาแค่ 0 ถึง 20 วิ แล้วหยุดอัปเดตคลื่น
	  	  		  if (signal_time <= 20.0f) {

	  	  			  t_plot = signal_time;

	  	  			  // คำนวณ Input
	  	  			  float32_t v_sin = sineGenerator(signal_time);
	  	  			  float32_t v_ramp = rampGenerator(signal_time);
	  	  			 //vin_Sine = v_sin; // เอาไว้เช็คใน CubeMonitor
	  	  			  vin_Ramp = v_ramp;

	  	  			  // ----------------------------------------------------
	  	  			  // 🌟 คำนวณ HIL (Sine & Ramp) ตามสวิตช์ที่ตั้งไว้
	  	  			  // ----------------------------------------------------

	  	  			  // [1] Forward Euler
	  	  			  if (enable_fwd) {
	  	  				  forwardEuler(v_sin, &stFwdSin);
	  	  				  forwardEuler(v_ramp, &stFwdRmp);
	  	  				  fwdOmega_Sine = stFwdSin.currentOmega;
	  	  				  fwdOmega_Ramp = stFwdRmp.currentOmega;
	  	  			  } else {
	  	  				  fwdOmega_Sine = 0.0f;
	  	  				  fwdOmega_Ramp = 0.0f;
	  	  			  }

	  	  			  // [2] Backward Euler
	  	  			  if (enable_bwd) {
	  	  				  backwardEuler(v_sin, &stBwdSin);
	  	  				  backwardEuler(v_ramp, &stBwdRmp);
	  	  				  bwdOmega_Sine = stBwdSin.currentOmega;
	  	  				  bwdOmega_Ramp = stBwdRmp.currentOmega;
	  	  			  } else {
	  	  				  bwdOmega_Sine = 0.0f;
	  	  				  bwdOmega_Ramp = 0.0f;
	  	  			  }

	  	  			  // [3] Tustin
	  	  			  if (enable_tus) {
	  	  				  tustin(v_sin, &stTusSin);
	  	  				  tustin(v_ramp, &stTusRmp);
	  	  				  tusOmega_Sine = stTusSin.currentOmega;
	  	  				  tusOmega_Ramp = stTusRmp.currentOmega;
	  	  			  } else {
	  	  				  tusOmega_Sine = 0.0f;
	  	  				  tusOmega_Ramp = 0.0f;
	  	  			  }

	  	  			  // ขยับเวลาไปข้างหน้า 1 Step
	  	  			  signal_time += TsInS;

	  	  		  } else {
	  	  			  // 🌟 เกิน 20 วิ ปล่อยค่าเป็น 0 ยาวๆ
	  	  			  fwdOmega_Sine = bwdOmega_Sine = tusOmega_Sine = 0.0f;
	  	  			  fwdOmega_Ramp = bwdOmega_Ramp = tusOmega_Ramp = 0.0f;
	  	  		  }
	  	  	  }
	  	    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float32_t sineGenerator(float32_t t_sec){
	// ของจริงมันมักจะเป็น sin() ปกติ ไม่ต้องมีลบข้างหน้าหรอกครับ
	float32_t angle = 2.0f * 3.14159f * sineFreq * t_sec;
	return amplitude * arm_sin_f32(angle);
}

float32_t rampGenerator(float32_t t_sec){
	// 🌟 ปรับให้กราฟเริ่มขึ้นตอน t = 1 (Start time 1) ตามโจทย์
	if(t_sec < ramp_start_time){
		return 0.0f;
	} else {
		return slope * (t_sec - ramp_start_time);
	}
}

void forwardEuler(float32_t Vin, MotorState_t *s){
	s->v2 = s->v1;
	s->v1 = s->v;
	s->v  = Vin;
	s->w2 = s->w1;
	s->w1 = s->currentOmega;
	float32_t a0 = (Km*TsInS*TsInS)/A;
	float32_t a1 = (((C*TsInS*TsInS)-(TsInS*B)+A))/A;
	float32_t a2 = ((TsInS*B)-(2*A))/A;
	s->currentOmega = ((a0*s->v2)-(a1*s->w2)-(a2*s->w1));
}

void backwardEuler(float32_t Vin, MotorState_t *s){
	s->v  = Vin;
	s->w2 = s->w1;
	s->w1 = s->currentOmega;
	float32_t den = (A+(TsInS*B)+(C*TsInS*TsInS));
	float32_t a0 = (A)/den;
	float32_t a1 = ((2*A)+(TsInS*B))/den;
	s->currentOmega = (s->v*Km*TsInS*TsInS/den)-(a0*s->w2)+(a1*s->w1);
}

void tustin(float32_t Vin, MotorState_t *s){
	s->v2 = s->v1;
	s->v1 = s->v;
	s->v  = Vin;
	s->w2 = s->w1;
	s->w1 = s->currentOmega;
	float32_t den = ((4*A)+(2*TsInS*B)+(C*TsInS*TsInS));
	float32_t a0 = ((Km*TsInS*TsInS)*(s->v+(2*s->v1)+s->v2))/den;
	float32_t a1 = ((-8*A)+(TsInS*2*TsInS*C))/den;
	float32_t a2 = ((4*A)-(2*TsInS*B)+(C*TsInS*TsInS))/den;
	s->currentOmega = a0-(a2*s->w2)-(a1*s->w1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
