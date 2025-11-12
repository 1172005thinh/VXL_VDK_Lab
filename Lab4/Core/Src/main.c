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
uint32_t task_id_1 = 0;   // Store task IDs for dynamic deletion
uint32_t task_id_2 = 0;
uint32_t task_id_3 = 0;
uint32_t task_id_4 = 0;
uint32_t task_id_5 = 0;
uint32_t task_id_6 = 0;
uint32_t task_id_7 = 0;
uint8_t demo_phase = 0;    // Track which demo phase we're in
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(unsigned char number);
void oneshot_startup(void);
void oneshot_delete_task1(void);
void oneshot_delete_task7(void);
void oneshot_final_cleanup(void);
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

// ============================================================================
// ONE-SHOT TASKS - Execute once and automatically deleted
// ============================================================================

void oneshot_startup(void) {
    // Startup
    HAL_GPIO_WritePin(_13_GPIO_Port, _13_Pin, GPIO_PIN_SET);
    HAL_Delay(750);
    // printf("Startup complete - Time: %lu ms\n", SCH_Get_Time() * 10);
}

void oneshot_delete_task1(void) {
    // Delete task 1 after 5 seconds
    if (task_id_1 != 0) {
        SCH_Delete_Task(task_id_1);
        //HAL_GPIO_WritePin(_1_GPIO_Port, _1_Pin, GPIO_PIN_RESET); // Ensure LED is off
        // task_id_1 = 0;
        // printf("Task 1 deleted - Time: %lu ms\n", SCH_Get_Time() * 10);
    }
}

void oneshot_delete_task7(void) {
    // Delete task 7 after 10 seconds
    if (task_id_7 != 0) {
        SCH_Delete_Task(task_id_7);
        //HAL_GPIO_WritePin(_7_GPIO_Port, _7_Pin, GPIO_PIN_RESET); // Ensure LED is off
        //task_id_7 = 0;
        // printf("Task 7 deleted - Time: %lu ms\n", SCH_Get_Time() * 10);
    }
}

void oneshot_final_cleanup(void) {
    // After 20 seconds, delete all remaining tasks one by one
    // Check if task IDs are valid before deleting
    if (task_id_1 != 0) SCH_Delete_Task(task_id_1);
    if (task_id_2 != 0) SCH_Delete_Task(task_id_2);
    if (task_id_3 != 0) SCH_Delete_Task(task_id_3);
    if (task_id_4 != 0) SCH_Delete_Task(task_id_4);
    if (task_id_5 != 0) SCH_Delete_Task(task_id_5);
    if (task_id_6 != 0) SCH_Delete_Task(task_id_6);
    if (task_id_7 != 0) SCH_Delete_Task(task_id_7);

    // Turn off all LEDs
    //HAL_GPIO_WritePin(_1_GPIO_Port, _1_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_2_GPIO_Port, _2_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_3_GPIO_Port, _3_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_4_GPIO_Port, _4_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_5_GPIO_Port, _5_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_6_GPIO_Port, _6_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(_7_GPIO_Port, _7_Pin, GPIO_PIN_RESET);
    
    display7SEG(10);  // Turn off 7-seg
    demo_phase = 1;  // Enter sleep mode
    // printf("All tasks deleted, entering sleep mode - Time: %lu ms\n", SCH_Get_Time() * 10);
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
   * SCHEDULER INITIALIZATION - ADVANCED DEMO
   * ============================================================================
   * This demo showcases:
   * 1. Periodic tasks running at different intervals
   * 2. One-shot tasks that execute once and auto-delete
   * 3. Dynamic task deletion using SCH_Delete_Task()
   * 4. Dynamic task addition using SCH_Add_Task()
   * 5. Sleep mode when no tasks are scheduled
   * 
   * Timeline:
   * - 0s: All periodic tasks start + startup one-shot
   * - 5s: Task 1 (500ms) is deleted dynamically
   * - 10s: Task 7 (3500ms) is deleted dynamically
   * - 15s: Task 1 is re-added but with 1000ms period
   * - 20s: All tasks deleted, system enters sleep mode
   * ============================================================================
   */
  
  // Initialize the scheduler
  SCH_Init();
  
  // Start the timer interrupt (10ms period)
  HAL_TIM_Base_Start_IT(&htim2);
  
  // ========== ONE-SHOT TASKS (Period = 0) ==========
  // These execute once at specified delay and are automatically deleted
  SCH_Add_Task(oneshot_startup, 10, 0);           // Startup animation at 100ms
  SCH_Add_Task(oneshot_delete_task1, 500, 0);    // Delete task1 at 5s
  SCH_Add_Task(oneshot_delete_task7, 1100, 0);   // Delete task7 at 11s
  SCH_Add_Task(oneshot_final_cleanup, 2500, 0);  // Cleanup at 25s
  
  // ========== PERIODIC TASKS ==========
  // These run continuously until explicitly deleted
  SCH_Add_Task(blinkLED_SYS, 0, 50);             // System heartbeat
  task_id_1 = SCH_Add_Task(blinkPin_1, 0, 50);   // Store ID for deletion
  task_id_2 =SCH_Add_Task(blinkPin_2, 0, 100);
  task_id_3 = SCH_Add_Task(blinkPin_3, 0, 150);
  task_id_4 = SCH_Add_Task(blinkPin_4, 0, 200);
  task_id_5 = SCH_Add_Task(blinkPin_5, 0, 250);
  task_id_6 = SCH_Add_Task(blinkPin_6, 0, 300);
  task_id_7 = SCH_Add_Task(blinkPin_7, 0, 350);  // Store ID for deletion

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /**
     * =========================================================================
     * MAIN SCHEDULER LOOP WITH SLEEP MODE
     * =========================================================================
     * The main loop continuously dispatches tasks that are ready to run.
     * 
     * When demo_phase = 1 (all tasks deleted), the system enters sleep mode
     * to save power. The CPU will wake up every 10ms on timer interrupt but
     * will have no tasks to execute, demonstrating low-power operation.
     * 
     * Sleep mode benefits:
     * - Reduces power consumption when idle
     * - CPU can still wake on interrupts (timer, GPIO, etc.)
     * - Useful for battery-powered applications
     * =========================================================================
     */
    
    // Dispatch any ready tasks
    SCH_Dispatch_Tasks();
    
    // Enter sleep mode if all tasks are done (demo_phase = 1)
    if (demo_phase == 1) {        
        // Enter sleep mode (CPU halts until next interrupt)
        // __WFI() = Wait For Interrupt
        // CPU will wake every 10ms on timer interrupt, check for tasks, then sleep again
        __WFI();
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
