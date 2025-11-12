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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
#include "global.h"
#include <stdio.h>
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(unsigned char number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * ============================================================================
 * SCHEDULER DEMO TASKS
 * ============================================================================
 * These tasks demonstrate the scheduler's ability to run multiple periodic
 * tasks at different intervals. Each task toggles an LED to provide visual
 * feedback of the timing accuracy.
 * 
 * The tasks run at:
 * - 500ms (2 Hz) - PA15/Pin13 (blinkLED_SYS) - System status LED
 * - 500ms (2 Hz) - PA1 (blinkPin_1) - Task 1 indicator
 * - 1000ms (1 Hz) - PA2 (blinkPin_2) - Task 2 indicator
 * - 1500ms (0.67 Hz) - PA3 (blinkPin_3) - Task 3 indicator
 * - 2000ms (0.5 Hz) - PA4 (blinkPin_4) - Task 4 indicator
 * - 2500ms (0.4 Hz) - PA5 (blinkPin_5) - Task 5 indicator
 * - 3000ms (0.33 Hz) - PA6 (blinkPin_6) - Task 6 indicator
 * - 3500ms (0.29 Hz) - PA7 (blinkPin_7) - Task 7 indicator
 * 
 * Note: Multiple tasks can have the same period. They execute sequentially
 * in the same scheduler dispatch cycle.
 * 
 * 7-Segment Display Mapping (PB0-PB6):
 * PB0 (_14) = segment a    PB3 (_17) = segment d    PB6 (_20) = segment g
 * PB1 (_15) = segment b    PB4 (_18) = segment e
 * PB2 (_16) = segment c    PB5 (_19) = segment f
 * ============================================================================
 */

void blinkLED_SYS(void) {
    // Toggle LED_SYS on pin PA15 (pin 13)
    HAL_GPIO_TogglePin(_13_GPIO_Port, _13_Pin);
    // This task runs concurrently with blinkPin_1 at the same 500ms interval
    //printf("Task 500ms Pin13 - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_1(void) {
    // Toggle LED on pin PA1 (example)
    HAL_GPIO_TogglePin(_1_GPIO_Port, _1_Pin);
    display7SEG(1);
    // Print timestamp (if UART is configured)
    // printf("blinkPin_1 - Task 500ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_2(void) {
    // Toggle LED on pin PA2 (example)
    HAL_GPIO_TogglePin(_2_GPIO_Port, _2_Pin);
    display7SEG(2);
    // printf("blinkPin_2 - Task 1000ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_3(void) {
    // Toggle LED on pin PA3 (example)
    HAL_GPIO_TogglePin(_3_GPIO_Port, _3_Pin);
    display7SEG(3);
    // printf("blinkPin_3 - Task 1500ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_4(void) {
    // Toggle LED on pin PA4 (example)
    HAL_GPIO_TogglePin(_4_GPIO_Port, _4_Pin);
    display7SEG(4);
    // printf("blinkPin_4 - Task 2000ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_5(void) {
    // Toggle LED on pin PA5 (example)
    HAL_GPIO_TogglePin(_5_GPIO_Port, _5_Pin);
    display7SEG(5);
    // printf("blinkPin_5 - Task 2500ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_6(void) {
    // Toggle LED on pin PA6 (example)
    HAL_GPIO_TogglePin(_6_GPIO_Port, _6_Pin);
    display7SEG(6);
    // printf("blinkPin_6 - Task 3000ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void blinkPin_7(void) {
    // Toggle LED on pin PA7 (example)
    HAL_GPIO_TogglePin(_7_GPIO_Port, _7_Pin);
    display7SEG(7);
    // printf("blinkPin_7 - Task 3500ms - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void display7SEG(unsigned char number) {
    uint8_t anode7SEG_map[11] = {
        0b0000001, //0
        0b1001111, //1
        0b0010010, //2
        0b0000110, //3
        0b1001100, //4
        0b0100100, //5
        0b0100000, //6
        0b0001111, //7
        0b0000000, //8
        0b0000100, //9
        0b1111111  //All segments off
    };
    
    // Display the current number on 7-segment (PB0-PB6: segments a-g)
    HAL_GPIO_WritePin(_14_GPIO_Port, _14_Pin, (anode7SEG_map[number] & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // a
    HAL_GPIO_WritePin(_15_GPIO_Port, _15_Pin, (anode7SEG_map[number] & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // b
    HAL_GPIO_WritePin(_16_GPIO_Port, _16_Pin, (anode7SEG_map[number] & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // c
    HAL_GPIO_WritePin(_17_GPIO_Port, _17_Pin, (anode7SEG_map[number] & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // d
    HAL_GPIO_WritePin(_18_GPIO_Port, _18_Pin, (anode7SEG_map[number] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // e
    HAL_GPIO_WritePin(_19_GPIO_Port, _19_Pin, (anode7SEG_map[number] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // f
    HAL_GPIO_WritePin(_20_GPIO_Port, _20_Pin, (anode7SEG_map[number] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // g
    // printf("display7SEG - Current Number: %d - Time: %lu ms\n", number, SCH_Get_Time() * 10);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  /**
   * ============================================================================
   * SCHEDULER INITIALIZATION
   * ============================================================================
   * 1. Initialize the scheduler data structures
   * 2. Start the 10ms timer interrupt (TIM2)
   * 3. Add 7 periodic tasks (3 at 500ms for LED_SYS, 7-seg, and Task1)
   * 
   * Timer Configuration:
   * - Clock: 8 MHz (HSI)
   * - Prescaler: 7999 (÷8000)
   * - Period: 9 (counts 0-9)
   * - Result: 8MHz / 8000 / 10 = 100 Hz (10ms tick)
   * 
   * Multiple tasks can share the same period - they execute sequentially
   * 
   * Task Schedule:
   * - Every 500ms: LED_SYS blinks, 7-seg counts (1→5), Pin1 blinks
   * - Every 1000ms: Pin2 blinks (also every 2nd 7-seg change)
   * - Every 1500ms: Pin3 blinks (also every 3rd 7-seg change)
   * - Every 2000ms: Pin4 blinks (also every 4th 7-seg change)
   * - Every 2500ms: Pin5 blinks (also every 5th 7-seg change)
   * - Every 3000ms: Pin6 blinks (also every 6th 7-seg change)
   * - Every 3500ms: Pin7 blinks (also every 7th 7-seg change)
   * ============================================================================
   */
  
  // Initialize the scheduler
  SCH_Init();
  
  // Start the timer interrupt (10ms period)
  HAL_TIM_Base_Start_IT(&htim2);
  
  // Add periodic tasks
  // Task IDs are returned but can be stored if needed for deletion later
  // Format: SCH_Add_Task(function, initial_delay, period)
  // Note: All times are in ticks (1 tick = 10ms)
  
  SCH_Add_Task(blinkLED_SYS, 0, 50);  // 500ms = 50 ticks (50 * 10ms)
  SCH_Add_Task(blinkPin_1, 0, 50);    // 500ms = 50 ticks
  SCH_Add_Task(blinkPin_2, 0, 100);  // 1000ms = 100 ticks
  SCH_Add_Task(blinkPin_3, 0, 150);  // 1500ms = 150 ticks
  SCH_Add_Task(blinkPin_4, 0, 200);  // 2000ms = 200 ticks
  SCH_Add_Task(blinkPin_5, 0, 250);  // 2500ms = 250 ticks
  SCH_Add_Task(blinkPin_6, 0, 300);  // 3000ms = 300 ticks
  SCH_Add_Task(blinkPin_7, 0, 350);  // 3500ms = 350 ticks

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /**
     * =========================================================================
     * MAIN SCHEDULER LOOP
     * =========================================================================
     * The main loop continuously dispatches tasks that are ready to run.
     * 
     * - SCH_Dispatch_Tasks() checks all tasks with RunMe flag set
     * - Executes ready tasks
     * - Re-schedules periodic tasks
     * - Deletes one-shot tasks
     * 
     * Optional: Add __WFI() to enter low power mode when no tasks are ready
     * =========================================================================
     */
    SCH_Dispatch_Tasks();
    
    // Optional: Enter low power mode if no tasks are ready
    // __WFI(); // Wait For Interrupt
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
