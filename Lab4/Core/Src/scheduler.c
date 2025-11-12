/*
 * scheduler.c
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 *  
 *  PROTEUS-COMPATIBLE VERSION
 *  Uses static array instead of dynamic memory allocation (malloc/free)
 *  to avoid simulation crashes
 */

#include "scheduler.h"
#include "global.h"

// Static task array (Proteus-compatible, no malloc needed)
static sTask SCH_tasks_G[SCH_MAX_TASKS];

// Task counter for generating unique IDs
static uint32_t task_id_counter = 0;

// System time counter (in ticks)
static uint32_t system_time_ticks = 0;

/**
 * @brief Initialize the scheduler
 * Sets up the task array as empty
 */
void SCH_Init(void) {
    uint8_t i;
    for (i = 0; i < SCH_MAX_TASKS; i++) {
        SCH_tasks_G[i].pTask = NULL;
        SCH_tasks_G[i].Delay = 0;
        SCH_tasks_G[i].Period = 0;
        SCH_tasks_G[i].RunMe = 0;
        SCH_tasks_G[i].TaskID = 0;
    }
    
    task_id_counter = 0;
    system_time_ticks = 0;
    Error_code_G = ERROR_SCH_OK;
}

/**
 * @brief Update function called from timer interrupt (every 10ms)
 * Updates all task delays - O(n) but simple and Proteus-compatible
 */
void SCH_Update(void) {
    uint8_t i;
    
    // Increment system time
    system_time_ticks++;
    
    // Check for overflow (optional)
    if (system_time_ticks == 0) {
        Error_code_G = ERROR_TICK_OVERFLOW;
    }
    
    // Update all tasks
    for (i = 0; i < SCH_MAX_TASKS; i++) {
        // Check if task exists
        if (SCH_tasks_G[i].pTask != NULL) {
            if (SCH_tasks_G[i].Delay > 0) {
                SCH_tasks_G[i].Delay--;
            }
            
            // If delay has reached 0, mark task as ready
            if (SCH_tasks_G[i].Delay == 0) {
                SCH_tasks_G[i].RunMe = 1;
            }
        }
    }
}

/**
 * @brief Dispatch tasks that are ready to run
 * Executes tasks with RunMe flag set and handles periodic tasks
 */
void SCH_Dispatch_Tasks(void) {
    uint8_t i;
    
    // Go through the task array
    for (i = 0; i < SCH_MAX_TASKS; i++) {
        // Check if task exists and is ready to run
        if (SCH_tasks_G[i].pTask != NULL && SCH_tasks_G[i].RunMe > 0) {
            // Execute the task
            (*SCH_tasks_G[i].pTask)();
            
            // Clear the RunMe flag
            SCH_tasks_G[i].RunMe = 0;
            
            // Handle periodic vs one-shot tasks
            if (SCH_tasks_G[i].Period > 0) {
                // Periodic task - reset delay
                SCH_tasks_G[i].Delay = SCH_tasks_G[i].Period;
            } else {
                // One-shot task - delete it
                SCH_tasks_G[i].pTask = NULL;
                SCH_tasks_G[i].TaskID = 0;
            }
        }
    }
}

/**
 * @brief Add a new task to the scheduler
 * @param pFunction Pointer to the function to be executed
 * @param DELAY Initial delay in ticks before first execution
 * @param PERIOD Period in ticks for periodic execution (0 for one-shot)
 * @return Task ID, or 0 if failed
 * 
 * Proteus-compatible version using static array
 */
uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD) {
    uint8_t i;
    
    // Find an empty slot
    for (i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].pTask == NULL) {
            // Found empty slot
            SCH_tasks_G[i].pTask = pFunction;
            SCH_tasks_G[i].Delay = DELAY;
            SCH_tasks_G[i].Period = PERIOD;
            SCH_tasks_G[i].RunMe = 0;
            SCH_tasks_G[i].TaskID = ++task_id_counter;
            
            return SCH_tasks_G[i].TaskID;
        }
    }
    
    // No empty slot found
    Error_code_G = ERROR_SCH_TOO_MANY_TASKS;
    return 0;
}

/**
 * @brief Delete a task from the scheduler
 * @param TASK_ID ID of the task to delete
 * @return 1 if successful, 0 if task not found
 */
uint8_t SCH_Delete_Task(uint32_t TASK_ID) {
    uint8_t i;
    
    // Search for task with matching ID
    for (i = 0; i < SCH_MAX_TASKS; i++) {
        if (SCH_tasks_G[i].TaskID == TASK_ID && SCH_tasks_G[i].pTask != NULL) {
            // Found the task to delete
            SCH_tasks_G[i].pTask = NULL;
            SCH_tasks_G[i].TaskID = 0;
            SCH_tasks_G[i].Delay = 0;
            SCH_tasks_G[i].Period = 0;
            SCH_tasks_G[i].RunMe = 0;
            
            return 1; // Success
        }
    }
    
    // Task not found
    Error_code_G = ERROR_SCH_CANNOT_DELETE_TASK;
    return 0;
}

/**
 * @brief Get current system time in ticks
 * @return Current system time (10ms ticks)
 */
uint32_t SCH_Get_Time(void) {
    return system_time_ticks;
}

