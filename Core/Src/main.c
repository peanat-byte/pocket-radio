/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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

	// Encoder to tune channel
	#define ENC_DELAY 10
	#define ENC_POS_MAX 200
	uint32_t prev_enc_time = 0;  // Time of the last interrupt, we only need one because the signals are in quadrature
	uint8_t stable_A = 0, stable_B = 0;
	uint8_t encoder_state = 0;
	volatile int8_t encoder_position = 0;
	volatile int8_t channel_changed = 0;

	#define CHANNEL_MIN 88.0
	#define CHANNEL_MAX 108.0
	float channel = CHANNEL_MIN;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* Power on Radio module by setting RST high */
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (channel_changed) {
		  channel = CHANNEL_MIN + (float)encoder_position;

		  // LED indicators when you reach the min/max channels
		  if (channel <= CHANNEL_MIN) {
			  HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_RESET);
		  }
		  else if (channel >= CHANNEL_MAX) {
			  HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_SET);
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOA, LED_1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, LED_2_Pin, GPIO_PIN_RESET);
		  }

		  // Send command to radio receiver to change channel

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SDIO_Pin|SCLK_Pin|SENB_Pin|LED_1_Pin
                          |LED_2_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SDIO_Pin SCLK_Pin SENB_Pin LED_1_Pin
                           LED_2_Pin RST_Pin */
  GPIO_InitStruct.Pin = SDIO_Pin|SCLK_Pin|SENB_Pin|LED_1_Pin
                          |LED_2_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_B_Pin ENC_A_Pin BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = ENC_B_Pin|ENC_A_Pin|BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == ENC_A_Pin || GPIO_Pin == ENC_B_Pin) {
	  uint32_t current_time = HAL_GetTick();
	  if ((current_time - prev_enc_time) < ENC_DELAY) {
		  return;
	  }
	  prev_enc_time = current_time;

	  uint8_t current_A = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  uint8_t current_B = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

	  // If the state has changed
	  if (current_A != stable_A || current_B != stable_B) {
		  stable_A = current_A;
		  stable_B = current_B;

		  // Use Gray code to determine direction (it goes 00 01 11 10)
		  uint8_t new_state = (stable_A << 1) | stable_B;
		  int8_t direction = 0;

		  switch (encoder_state) {
			  case 0:
				  if (new_state == 1) direction = 1;
				  if (new_state == 2) direction = -1;
				  break;
			  case 1:
				  if (new_state == 3) direction = 1;
				  if (new_state == 0) direction = -1;
				  break;
			  case 2:
				  if (new_state == 0) direction = 1;
				  if (new_state == 3) direction = -1;
				  break;
			  case 3:
				  if (new_state == 2) direction = 1;
				  if (new_state == 1) direction = -1;
				  break;
		  }

		  encoder_position += direction;
		  if (encoder_position > ENC_POS_MAX) {
			  encoder_position = ENC_POS_MAX;
		  }
		  else if (encoder_position < 0) {
			  encoder_position = 0;
		  }
		  encoder_state = new_state;

		  channel_changed = 1;
	  }
  }
  else {
	  __NOP();
  }
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

#ifdef  USE_FULL_ASSERT
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
