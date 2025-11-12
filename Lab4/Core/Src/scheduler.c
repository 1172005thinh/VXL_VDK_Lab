/*
 * scheduler.c
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 */

#include "scheduler.h"
#include "global.h"
#include <stdio.h>

// Linked list head pointer
static sTask *task_list_head = NULL;

// Task counter for generating unique IDs
static uint32_t task_id_counter = 0;

// System time counter (in ticks)
static uint32_t system_time_ticks = 0;

void SCH_Init(void) {
    task_list_head = NULL;
    task_id_counter = 0;
    system_time_ticks = 0;
    Error_code_G = ERROR_SCH_OK;
}

void SCH_Update(void) {
    // Increment system time
    system_time_ticks++;
    
    // Check for overflow (optional)
    if (system_time_ticks == 0) {
        Error_code_G = ERROR_TICK_OVERFLOW;
    }
    
    // Update the first task's delay if list is not empty
    if (task_list_head != NULL) {
        if (task_list_head->Delay > 0) {
            task_list_head->Delay--;
        }
        
        // If delay has reached 0, mark task as ready
        if (task_list_head->Delay == 0) {
            task_list_head->RunMe = 1;
        }
    }
}

void SCH_Dispatch_Tasks(void) {
    sTask *current_task = task_list_head;
    sTask *prev_task = NULL;
    
    while (current_task != NULL) {
        // Check if task is ready to run
        if (current_task->RunMe > 0) {
            // Execute the task
            (*current_task->pTask)();
            
            // Clear the RunMe flag
            current_task->RunMe = 0;
            
            // Handle periodic vs one-shot tasks
            if (current_task->Period > 0) {
                // Periodic task - reset delay and re-insert into list
                current_task->Delay = current_task->Period;
                
                // Remove from current position
                sTask *task_to_reinsert = current_task;
                if (prev_task == NULL) {
                    task_list_head = current_task->next;
                } else {
                    prev_task->next = current_task->next;
                }
                
                // Re-insert in sorted position
                sTask *insert_prev = NULL;
                sTask *insert_current = task_list_head;
                uint32_t accumulated_delay = 0;
                
                while (insert_current != NULL && 
                       accumulated_delay + insert_current->Delay < task_to_reinsert->Delay) {
                    accumulated_delay += insert_current->Delay;
                    insert_prev = insert_current;
                    insert_current = insert_current->next;
                }
                
                // Adjust delays
                task_to_reinsert->Delay -= accumulated_delay;
                if (insert_current != NULL) {
                    insert_current->Delay -= task_to_reinsert->Delay;
                }
                
                // Insert into list
                task_to_reinsert->next = insert_current;
                if (insert_prev == NULL) {
                    task_list_head = task_to_reinsert;
                } else {
                    insert_prev->next = task_to_reinsert;
                }
                
                // Continue from the next task
                current_task = (prev_task == NULL) ? task_list_head : prev_task->next;
                
            } else {
                // One-shot task - delete it
                sTask *task_to_delete = current_task;
                if (prev_task == NULL) {
                    task_list_head = current_task->next;
                    current_task = task_list_head;
                } else {
                    prev_task->next = current_task->next;
                    current_task = prev_task->next;
                }
                free(task_to_delete);
            }
        } else {
            // Move to next task
            prev_task = current_task;
            current_task = current_task->next;
        }
    }
}

uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD) {
    // Allocate memory for new task
    sTask *new_task = (sTask *)malloc(sizeof(sTask));
    
    if (new_task == NULL) {
        Error_code_G = ERROR_SCH_TOO_MANY_TASKS;
        return 0;
    }
    
    // Initialize task
    new_task->pTask = pFunction;
    new_task->Delay = DELAY;
    new_task->Period = PERIOD;
    new_task->RunMe = 0;
    new_task->TaskID = ++task_id_counter;
    new_task->next = NULL;
    
    // Insert task into sorted linked list (sorted by delay)
    sTask *current = task_list_head;
    sTask *prev = NULL;
    uint32_t accumulated_delay = 0;
    
    // Find insertion point
    while (current != NULL && accumulated_delay + current->Delay < DELAY) {
        accumulated_delay += current->Delay;
        prev = current;
        current = current->next;
    }
    
    // Adjust delays (delta encoding)
    new_task->Delay = DELAY - accumulated_delay;
    if (current != NULL) {
        current->Delay -= new_task->Delay;
    }
    
    // Insert into list
    new_task->next = current;
    if (prev == NULL) {
        // Insert at head
        task_list_head = new_task;
    } else {
        // Insert after prev
        prev->next = new_task;
    }
    
    return new_task->TaskID;
}

uint8_t SCH_Delete_Task(uint32_t TASK_ID) {
    sTask *current = task_list_head;
    sTask *prev = NULL;
    
    // Search for task with matching ID
    while (current != NULL) {
        if (current->TaskID == TASK_ID) {
            // Found the task to delete
            
            // Adjust next task's delay if it exists
            if (current->next != NULL) {
                current->next->Delay += current->Delay;
            }
            
            // Remove from list
            if (prev == NULL) {
                // Deleting head
                task_list_head = current->next;
            } else {
                prev->next = current->next;
            }
            
            // Free memory
            free(current);
            
            return 1; // Success
        }
        
        prev = current;
        current = current->next;
    }
    
    // Task not found
    Error_code_G = ERROR_SCH_CANNOT_DELETE_TASK;
    return 0;
}

uint32_t SCH_Get_Time(void) {
    return system_time_ticks;
}

