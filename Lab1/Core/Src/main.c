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
	//PA1 to PA12
	{_1_GPIO_Port, _1_Pin},   	// indexPin = 1
    {_2_GPIO_Port, _2_Pin},  	// indexPin = 2
    {_3_GPIO_Port, _3_Pin},  	// indexPin = 3
    {_4_GPIO_Port, _4_Pin},   	// indexPin = 4
    {_5_GPIO_Port, _5_Pin},   	// indexPin = 5
    {_6_GPIO_Port, _6_Pin},   	// indexPin = 6
    {_7_GPIO_Port, _7_Pin},   	// indexPin = 7
    {_8_GPIO_Port, _8_Pin},   	// indexPin = 8
    {_9_GPIO_Port, _9_Pin},   	// indexPin = 9
    {_10_GPIO_Port, _10_Pin},  	// indexPin = 10
    {_11_GPIO_Port, _11_Pin},  	// indexPin = 11
    {_12_GPIO_Port, _12_Pin},  	// indexPin = 12
	//PA13 to PA14 are reserved
	//PA15 for LED-SYS
	{_13_GPIO_Port, _13_Pin},  	// indexPin = 13
	//PB0 to PB15
    {_14_GPIO_Port, _14_Pin},  	// indexPin = 14
    {_15_GPIO_Port, _15_Pin},  	// indexPin = 15
    {_16_GPIO_Port, _16_Pin},  	// indexPin = 16
    {_17_GPIO_Port, _17_Pin},  	// indexPin = 17
    {_18_GPIO_Port, _18_Pin},  	// indexPin = 18
    {_19_GPIO_Port, _19_Pin},  	// indexPin = 19
    {_20_GPIO_Port, _20_Pin},  	// indexPin = 20
    {_21_GPIO_Port, _21_Pin},  	// indexPin = 21
    {_22_GPIO_Port, _22_Pin},  	// indexPin = 22
    {_23_GPIO_Port, _23_Pin},  	// indexPin = 23
    {_24_GPIO_Port, _24_Pin},  	// indexPin = 24
	{_25_GPIO_Port, _25_Pin},  	// indexPin = 25
    {_26_GPIO_Port, _26_Pin},  	// indexPin = 26
    {_27_GPIO_Port, _27_Pin},  	// indexPin = 27
	{_28_GPIO_Port, _27_Pin},  	// indexPin = 28
    {_29_GPIO_Port, _29_Pin},  	// indexPin = 29
};

const uint8_t segMapAnode[10] = {
	//abcdefg
	0b0000001, //0
	0b1001111, //1
	0b0010010, //2
	0b0000110, //3
	0b1001100, //4
	0b0100100, //5
	0b0100000, //6
	0b0001111, //7
	0b0000000, //8
	0b0000100  //9
};

void togglePin(int indexPin, bool state) {
	int idx = indexPin - 1;
	//safety check -> time_1_4 == time_2_5 + time_3_6
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
			togglePin(indexPin_1, 1);
			togglePin(indexPin_2, 0);
			togglePin(indexPin_3, 0);
			break;

			case 1: // YELLOW
			togglePin(indexPin_1, 0);
			togglePin(indexPin_2, 1);
			togglePin(indexPin_3, 0);
			break;

			case 2: // GREEN
			togglePin(indexPin_1, 0);
			togglePin(indexPin_2, 0);
			togglePin(indexPin_3, 1);
			break;

			default: // safety fallback -> all OFF and reset
			togglePin(indexPin_1, 0);
			togglePin(indexPin_2, 0);
			togglePin(indexPin_3, 0);
			state = 0;
			counter = 0;
			break;
		}
	
		HAL_Delay(1000);
		counter++;

		if (state == 0 && counter >= time_1) {
			state = 1;
			counter = 0;
		} else if (state == 1 && counter >= time_2) {
			state = 2;
			counter = 0;
		} else if (state == 2 && counter >= time_3) {
			state = 0;
			counter = 0;
		}
	}
}

void toggle6Pin(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6,
				bool state_1, bool state_2, bool state_3, bool state_4, bool state_5, bool state_6) {
	togglePin(indexPin_1, state_1);
	togglePin(indexPin_2, state_2);
	togglePin(indexPin_3, state_3);
	togglePin(indexPin_4, state_4);
	togglePin(indexPin_5, state_5);
	togglePin(indexPin_6, state_6);
}

void toggle7Pin(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6, int indexPin_7,
				bool state_1, bool state_2, bool state_3, bool state_4, bool state_5, bool state_6, bool state_7) {
	togglePin(indexPin_1, state_1);
	togglePin(indexPin_2, state_2);
	togglePin(indexPin_3, state_3);
	togglePin(indexPin_4, state_4);
	togglePin(indexPin_5, state_5);
	togglePin(indexPin_6, state_6);
	togglePin(indexPin_7, state_7);
}

void traffic3_4(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6,
				int time_1_4, int time_2_5, int time_3_6,
				int state, int counter) {
	//safety check -> time_1_4 == time_2_5 + time_3_6
	if (time_1_4 != time_2_5 + time_3_6) {
		return;
	}
	while (1)
	{
		switch (state)
		{
			case 0: // RED GREEN
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 1, 0, 0, 0, 0, 1);
			break;

			case 1: // RED YELLOW
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 1, 0, 0, 0, 1, 0);
			break;

			case 2: // GREEN RED
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 0, 1, 1, 0, 0);
			break;
			
			case 3: // YELLOW RED
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 1, 0, 1, 0, 0);
			break;

			default: // safety fallback -> all OFF and reset
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 0, 0, 0, 0, 0);
			state = 0;
			counter = 0;
			break;
		}
		
		HAL_Delay(1000);
		counter++;

		if (state == 0 && counter >= time_3_6) {
			state = 1;
			counter = 0;
		} else if (state == 1 && counter >= time_2_5) {
			state = 2;
			counter = 0;
		} else if (state == 2 && counter >= time_1_4) {
			state = 3;
			counter = 0;
		} else if (state == 3 && counter >= time_2_5) {
			state = 0;
			counter = 0;
		}
	}
}

void seg7Anode_1(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6, int indexPin_7, 
				const uint8_t state[10], int num,
				int time) {
	toggle7Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, indexPin_7, 1, 1, 1, 1, 1, 1, 1);
	togglePin(indexPin_1, state[num] & 0b1000000);
	togglePin(indexPin_2, state[num] & 0b0100000);
	togglePin(indexPin_3, state[num] & 0b0010000);
	togglePin(indexPin_4, state[num] & 0b0001000);
	togglePin(indexPin_5, state[num] & 0b0000100);
	togglePin(indexPin_6, state[num] & 0b0000010);
	togglePin(indexPin_7, state[num] & 0b0000001);
	//HAL_Delay(time*1000);
}

void seg7Anode_2(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6, int indexPin_7, 
				const uint8_t state[10], int num,
				int time) {
	toggle7Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, indexPin_7, 1, 1, 1, 1, 1, 1, 1);
	togglePin(indexPin_1, state[num] & 0b1000000);
	togglePin(indexPin_2, state[num] & 0b0100000);
	togglePin(indexPin_3, state[num] & 0b0010000);
	togglePin(indexPin_4, state[num] & 0b0001000);
	togglePin(indexPin_5, state[num] & 0b0000100);
	togglePin(indexPin_6, state[num] & 0b0000010);
	togglePin(indexPin_7, state[num] & 0b0000001);
	//HAL_Delay(time*1000);
}

void display7SEG(int num) {
	//safety check -> 0 <= num <= 9
	if (num < 0 || num > 9) {
		return;
	}
	seg7Anode_1(14, 15, 16, 17, 18, 19, 20, segMapAnode, num, 1);
	HAL_Delay(1000);
}

void traffic3_7seg_4(int indexPin_1, int indexPin_2, int indexPin_3, int indexPin_4, int indexPin_5, int indexPin_6,
					int indexPin_7, int indexPin_8, int indexPin_9, int indexPin_10, int indexPin_11, int indexPin_12, int indexPin_13,
					int indexPin_14, int indexPin_15, int indexPin_16, int indexPin_17, int indexPin_18, int indexPin_19, int indexPin_20,
					int time_1_4, int time_2_5, int time_3_6,
					int state, int counter) {
	//safety check -> time_1_4 == time_2_5 + time_3_6
	if (time_1_4 != time_2_5 + time_3_6) {
		return;
	}
	bool led_sys = 0;
	while (1)
	{		
		if (led_sys == 0) {
			togglePin(13, 1);
			led_sys = 1;
		} else {
			togglePin(13, 0);
			led_sys = 0;
		}
		switch (state)
		{
			case 0: // RED GREEN
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 1, 0, 0, 0, 0, 1);
			seg7Anode_1(indexPin_7, indexPin_8, indexPin_9, indexPin_10, indexPin_11, indexPin_12, indexPin_13, segMapAnode, time_1_4 - counter - 1, 1);
			seg7Anode_2(indexPin_14, indexPin_15, indexPin_16, indexPin_17, indexPin_18, indexPin_19, indexPin_20, segMapAnode, time_3_6 - counter - 1 , 1);
			break;

			case 1: // RED YELLOW
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 1, 0, 0, 0, 1, 0);
			seg7Anode_1(indexPin_7, indexPin_8, indexPin_9, indexPin_10, indexPin_11, indexPin_12, indexPin_13, segMapAnode, time_2_5 - counter - 1, 1);
			seg7Anode_2(indexPin_14, indexPin_15, indexPin_16, indexPin_17, indexPin_18, indexPin_19, indexPin_20, segMapAnode, time_2_5 - counter - 1, 1);
			break;

			case 2: // GREEN RED
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 0, 1, 1, 0, 0);
			seg7Anode_1(indexPin_7, indexPin_8, indexPin_9, indexPin_10, indexPin_11, indexPin_12, indexPin_13, segMapAnode, time_3_6 - counter - 1, 1);
			seg7Anode_2(indexPin_14, indexPin_15, indexPin_16, indexPin_17, indexPin_18, indexPin_19, indexPin_20, segMapAnode, time_1_4 - counter - 1, 1);
			break;
			
			case 3: // YELLOW RED
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 1, 0, 1, 0, 0);
			seg7Anode_1(indexPin_7, indexPin_8, indexPin_9, indexPin_10, indexPin_11, indexPin_12, indexPin_13, segMapAnode, time_2_5 - counter - 1, 1);
			seg7Anode_2(indexPin_14, indexPin_15, indexPin_16, indexPin_17, indexPin_18, indexPin_19, indexPin_20, segMapAnode, time_2_5 - counter - 1, 1);
			break;

			default: // safety fallback -> all OFF and reset
			toggle6Pin(indexPin_1, indexPin_2, indexPin_3, indexPin_4, indexPin_5, indexPin_6, 0, 0, 0, 0, 0, 0);
			seg7Anode_1(indexPin_7, indexPin_8, indexPin_9, indexPin_10, indexPin_11, indexPin_12, indexPin_13, segMapAnode, 0, 1);
			seg7Anode_2(indexPin_14, indexPin_15, indexPin_16, indexPin_17, indexPin_18, indexPin_19, indexPin_20, segMapAnode, 0, 1);
			state = 0;
			counter = 0;
			break;
		}
		
		HAL_Delay(1000);
		counter++;

		if (state == 0 && counter >= time_3_6) {
			state = 1;
			counter = 0;
		} else if (state == 1 && counter >= time_2_5) {
			state = 2;
			counter = 0;
		} else if (state == 2 && counter >= time_3_6) {
			state = 3;
			counter = 0;
		} else if (state == 3 && counter >= time_2_5) {
			state = 0;
			counter = 0;
		}
	}
}

void seqClk() {	
	int pinMapClk[12] = {
		12, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11
	};
	bool led_sys = 0;	
	for (int i = 0; i <= 11; i++) {
		if (led_sys == 0) {
			togglePin(13, 1);
			led_sys = 1;
		} else {
			togglePin(13, 0);
			led_sys = 0;
		}
		for (int j = 0; j <= 11; j++) {
			togglePin(pinMapClk[j], 0);
		}
		togglePin(pinMapClk[i], 1);
		//Uncomment the line below to test clearAllClock() function
		//if (i == 6 || i == 2 || i == 10) clearAllClock();
		HAL_Delay(1000);
	}
}

void clearAllClock() {
	int pinMapClk[12] = {
		12, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11
	};
	for (int i = 0; i <= 11; i++) {
		togglePin(pinMapClk[i], 0);
	}
}

void setNumberOnClock(int num) {
	int pinMapClk[12] = {
		12, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11
	};
	togglePin(pinMapClk[num], 1);
}

void clearNumberOnClock(int num) {
	int pinMapClk[12] = {
		12, 1, 2, 3, 4, 5, 6, 7, 8, 9 ,10, 11
	};
	togglePin(pinMapClk[num], 0);
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
  bool led_sys = 0;
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
	/*
	traffic3_1(1, 2, 3, 5, 2, 3, status, counter);
	*/
	//Ex3
	/*
	traffic3_4(1, 2, 3, 4, 5, 6, 5, 2, 3, status, counter);
	*/
	//Ex4
	/*
	if (counter >= 10) {
		counter = 0;
	}
	display7SEG(counter++);
	*/
	//Ex5
	/*
	traffic3_7seg_4(1, 2 ,3 ,4 ,5, 6, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 5, 2, 3, status, counter);
	*/
	//Ex6
	/*	
	seqClk();
	*/
	//Ex7
	/*
	if (led_sys == 0) {
		togglePin(13, 1);
		led_sys = 1;
	} else {
		togglePin(13, 0);
		led_sys = 0;
	}
	clearAllClock();
	HAL_Delay(1000);
	*/
	//Ex8
	/*
	if (led_sys == 0) {
		togglePin(13, 1);
		led_sys = 1;
	} else {
		togglePin(13, 0);
		led_sys = 0;
	}
	setNumberOnClock(0);
	setNumberOnClock(2);
	setNumberOnClock(5);
	HAL_Delay(1000);
	*/
	//Ex9
	///*
	if (led_sys == 0) {
		togglePin(13, 1);
		led_sys = 1;
	} else {
		togglePin(13, 0);
		led_sys = 0;
	}
	setNumberOnClock(0);
	setNumberOnClock(2);
	setNumberOnClock(5);
	clearNumberOnClock(0);
	clearNumberOnClock(10);
	HAL_Delay(1000);
	//*/
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin
                          |_13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _14_Pin|_15_Pin|_16_Pin|_24_Pin
                          |_25_Pin|_26_Pin|_27_Pin|_28_Pin
                          |_29_Pin|_17_Pin|_18_Pin|_19_Pin
                          |_20_Pin|_21_Pin|_22_Pin|_23_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : _1_Pin _2_Pin _3_Pin _4_Pin
                           _5_Pin _6_Pin _7_Pin _8_Pin
                           _9_Pin _10_Pin _11_Pin _12_Pin
                           _13_Pin */
  GPIO_InitStruct.Pin = _1_Pin|_2_Pin|_3_Pin|_4_Pin
                          |_5_Pin|_6_Pin|_7_Pin|_8_Pin
                          |_9_Pin|_10_Pin|_11_Pin|_12_Pin
                          |_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : _14_Pin _15_Pin _16_Pin _24_Pin
                           _25_Pin _26_Pin _27_Pin _28_Pin
                           _29_Pin _17_Pin _18_Pin _19_Pin
                           _20_Pin _21_Pin _22_Pin _23_Pin */
  GPIO_InitStruct.Pin = _14_Pin|_15_Pin|_16_Pin|_24_Pin
                          |_25_Pin|_26_Pin|_27_Pin|_28_Pin
                          |_29_Pin|_17_Pin|_18_Pin|_19_Pin
                          |_20_Pin|_21_Pin|_22_Pin|_23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
