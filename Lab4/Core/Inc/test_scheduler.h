/*
 * test_scheduler.h
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 *  
 *  This file contains test functions to demonstrate scheduler capabilities
 */

#ifndef INC_TEST_SCHEDULER_H_
#define INC_TEST_SCHEDULER_H_

#include "scheduler.h"

// Test functions
void test_one_shot_task(void);
void test_multiple_timeouts(void);
void test_task_deletion(void);
void print_timestamp(void);

// Task functions for testing
void test_task_10ms(void);
void test_task_100ms(void);
void test_task_1000ms(void);

#endif /* INC_TEST_SCHEDULER_H_ */
