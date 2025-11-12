/*
 * test_scheduler.c
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 *  
 *  This file contains test functions to demonstrate scheduler capabilities
 */

#include "test_scheduler.h"
#include "main.h"
#include <stdio.h>

// Task IDs for tracking
static uint32_t task_id_10ms = 0;
static uint32_t task_id_100ms = 0;
static uint32_t task_id_1000ms = 0;

/**
 * @brief Test one-shot task (fires once after delay)
 * Demonstrates: SCH_Add_Task with Period = 0
 */
void test_one_shot_task(void) {
    // Add a one-shot task that fires after 2 seconds
    SCH_Add_Task(print_timestamp, 200, 0);
    // After 2 seconds, print_timestamp will execute once and be deleted
}

/**
 * @brief Test multiple concurrent timeouts
 * Demonstrates: Multiple tasks running at different intervals
 */
void test_multiple_timeouts(void) {
    // Add three tasks at different intervals
    task_id_10ms = SCH_Add_Task(test_task_10ms, 1, 1);      // Every 10ms
    task_id_100ms = SCH_Add_Task(test_task_100ms, 10, 10);  // Every 100ms
    task_id_1000ms = SCH_Add_Task(test_task_1000ms, 100, 100); // Every 1s
}

/**
 * @brief Test task deletion
 * Demonstrates: SCH_Delete_Task functionality
 */
void test_task_deletion(void) {
    // Delete the 10ms task
    if (SCH_Delete_Task(task_id_10ms)) {
        // Successfully deleted
        task_id_10ms = 0;
    }
}

/**
 * @brief Print current timestamp
 */
void print_timestamp(void) {
    uint32_t time_ms = SCH_Get_Time() * 10;
    // If UART is configured, you can use printf
    // printf("Timestamp: %lu ms\n", time_ms);
    
    // For now, toggle an LED to indicate execution
    HAL_GPIO_TogglePin(_6_GPIO_Port, _6_Pin);
}

/**
 * @brief Task that runs every 10ms
 */
void test_task_10ms(void) {
    // Toggle LED to show 10ms timing
    HAL_GPIO_TogglePin(_7_GPIO_Port, _7_Pin);
}

/**
 * @brief Task that runs every 100ms
 */
void test_task_100ms(void) {
    // Toggle LED to show 100ms timing
    HAL_GPIO_TogglePin(_8_GPIO_Port, _8_Pin);
}

/**
 * @brief Task that runs every 1000ms (1 second)
 */
void test_task_1000ms(void) {
    // Toggle LED to show 1s timing
    HAL_GPIO_TogglePin(_9_GPIO_Port, _9_Pin);
    
    // Example: Print timestamp every second
    // printf("Time: %lu ms\n", SCH_Get_Time() * 10);
}
