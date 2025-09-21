/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
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
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} PINMAP;

const PINMAP pinMap[] = {
	{_1_GPIO_Port,  _1_Pin},   // indexPin = 1
    {_2_GPIO_Port,  _2_Pin},   // indexPin = 2
    {_3_GPIO_Port,  _3_Pin},   // indexPin = 3
    {_4_GPIO_Port,  _4_Pin},   // indexPin = 4
    {_5_GPIO_Port,  _5_Pin},   // indexPin = 5
    {_6_GPIO_Port,  _6_Pin},   // indexPin = 6
    {_7_GPIO_Port,  _7_Pin},   // indexPin = 7
    {_8_GPIO_Port,  _8_Pin},   // indexPin = 8
    {_9_GPIO_Port,  _9_Pin},   // indexPin = 9
    {_10_GPIO_Port, _10_Pin},  // indexPin = 10
    {_11_GPIO_Port, _11_Pin},  // indexPin = 11
    {_12_GPIO_Port, _12_Pin},  // indexPin = 12
};

void togglePin(int indexPin, bool state) {
	int idx = indexPin - 1;
	if (idx < 0 || idx >= (sizeof(pinMap) / sizeof(pinMap[0]))) {
		return;
	}
	HAL_GPIO_WritePin(pinMap[idx].port, pinMap[idx].pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void traffic3_1(int indexPin_1, int indexPin_2, int indexPin_3, int time_1, int time_2, int time_3, int state, int counter) {
	while (1)
	{
		switch (state) {
			case 0: // RED
			togglePin(1, 1);
			togglePin(2, 0);
			togglePin(3, 0);
			break;

			case 1: // YELLOW
			togglePin(1, 0);
			togglePin(2, 1);
			togglePin(3, 0);
			break;

			case 2: // GREEN
			togglePin(1, 0);
			togglePin(2, 0);
			togglePin(3, 1);
			break;

			default: // safety fallback -> all OFF and reset
			togglePin(1, 0);
			togglePin(2, 0);
			togglePin(3, 0);
			state = 0;
			counter = 0;
			break;
		}
	
		HAL_Delay(1000);
		counter++;

		if (state == 0 && counter >= 5) {
			state = 1;
			counter = 0;
		} else if (state == 1 && counter >= 2) {
			state = 2;
			counter = 0;
		} else if (state == 2 && counter >= 3) {
			state = 0;
			counter = 0;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int status = 0;
  int counter = 0;
  while (1)
  {
	//Sample
	/*
	if (status == 0) {
		togglePin(1, 1);
		status = 1;
	} else {
		togglePin(1, 0);
		status = 0;
	}
	HAL_Delay(1000);
	*/
	//Ex1
	/*
	if (status == 0) {
		togglePin(1, 1);
		togglePin(2, 0);
		status = 1;
	} else {
		togglePin(1, 0);
		togglePin(2, 1);
		status = 0;
	}
	HAL_Delay(1000);
	*/
	//Ex2
	traffic3_1(1, 2, 3, 5, 2, 3, status, counter);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : _1_Pin _2_Pin _3_Pin _4_Pin
                           _5_Pin _6_Pin _7_Pin _8_Pin
                           _9_Pin _10_Pin _11_Pin _12_Pin */
  GPIO_InitStruct.Pin = _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
