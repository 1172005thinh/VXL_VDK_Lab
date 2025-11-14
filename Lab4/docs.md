# AI PROMPT

This file is particularly used for documenting AI prompts. Do not modify this file, I am the only one who can do that.

## Taget

We are going to desgin and implement a cooperate scheduler to accurately provide timeouts and trigger acitvities at the right time. You should follow the instructions in this file `Lab4/VXL_VDK_Lab4.pdf` to complete the lab.

**If you can not read PDF**, see the file `Lab4/VXL_VDK_Lab4.txt` for the same content.

## Requirements

Our system should have at least 4 funcs:

1. `void SCH_Update(void)`: This function will be updated the remaining time of each task that are added to a queue. It will be called in the interrupt timer, for example 10 ms.
2. `void SCH_Dispatch_Tasks(void)`: This function will get the task in the queue to run.
3. `uint32_t SCH_Add_Task(void (*pFunction)(), uint32_t DELAY, uint32_t PERIOD)`: This function is used to add a task to the queue. It should return an ID that is corresponding with the added task.
4. `uint8_t SCH_Delete_Task(uint32_t TASK_ID)`: This function is used to delete a task based on its ID.

You should add more functions if you think it is necessary.
Our main program must have 5 task running periodically in 0.5 sec, 1 sec, 1.5 sec, 2 sec, and 2.5 sec.

## Disqualifications

Only support a single timeout registered at a time.
Delivering callbacks in the wrong order.
O(n) searches in the SCH_Update function.
Interrupt freq greater than 10Hz, if our timer ticks regularly.

## Demo

A regular 10ms timer tick.
Register a timeout to fire a callback every 10ms.
Then print the value returned by get_time every time this callback is received.
Our timestamps should be at least accurate to the nearest 10ms.
Register another timeout at a different interval in addition to the 500ms running concurrently (i.e. demo more than one timeout registered at a time).
Before entering the main loop, set up a few calls to SCH_Add_Task. Make sure the delay used is long enough such that the loop is entered before these wake up. These callbacks should just print out the current timestamp as each delay expires.

## Suggestion

As O(n) search is not allowed in the update function, you can use a linked list to store the tasks. Make it O(1) to update the timeouts.

Project structure suggestion:

- Lab4
  - VXL_VDK_Lab4.pdf
  - VXL_VDK_Lab4.txt
  - Inc
    - scheduler.h --> stores the function prototypes and task struct
    - global.h --> stores global variables, such as ERROR_CODE
    - main.h --> stm32 generated file
    - ... other stm32 generated files
  - Src
    - scheduler.c --> implements the scheduler functions
    - global.c --> implements global variables
    - main.c --> stm32 generated file, implements main function
    - ... other stm32 generated files
  - docs.md
