/*
 * scheduler.h
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <stdint.h>
#include <stdlib.h>

// Maximum number of tasks in the scheduler
#define SCH_MAX_TASKS 40

// Task structure for the scheduler
typedef struct sTask {
    void (*pTask)(void);        // Function pointer to the task
    uint32_t Delay;             // Delay (ticks) until the task will run
    uint32_t Period;            // Period (ticks) for periodic tasks (0 = one-shot)
    uint8_t RunMe;              // Flag indicating task is ready to run
    uint32_t TaskID;            // Unique task identifier
    struct sTask *next;         // Pointer to next task in linked list
} sTask;

// Function prototypes
void SCH_Init(void);
void SCH_Update(void);
void SCH_Dispatch_Tasks(void);
uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD);
uint8_t SCH_Delete_Task(uint32_t TASK_ID);

// Utility functions
uint32_t SCH_Get_Time(void);

#endif /* INC_SCHEDULER_H_ */
