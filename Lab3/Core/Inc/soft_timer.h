/*
 * soft_timer.h
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#ifndef INC_SOFT_TIMER_H_
#define INC_SOFT_TIMER_H_

#include <stdbool.h>

// Timer interrupt cycle duration in milliseconds
// Change this value if you modify the hardware timer period
// Current: 10ms (Prescaler=7999, Period=9, Clock=8MHz)
#define TIMER_CYCLE_MS 10

extern bool timerLED_SYS_flag;
extern bool timer1_flag;
extern bool timer2_flag;
extern bool timer3_flag;

void setTimerLED_SYS(int duration);
void setTimer1(int duration);
void setTimer2(int duration);
void setTimer3(int duration);
void timerRun();

#endif /* INC_SOFT_TIMER_H_ */
