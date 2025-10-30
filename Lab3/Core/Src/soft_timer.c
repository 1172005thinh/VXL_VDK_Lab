/*
 * soft_timer.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "soft_timer.h"
#include <stdbool.h>

bool timerLED_SYS_flag = 0;
bool timer1_flag = 0;
bool timer2_flag = 0;
bool timer3_flag = 0;

int timerLED_SYS_counter = 0;
int timer1_counter = 0;
int timer2_counter = 0;
int timer3_counter = 0;

// Duration is in TIMER_CYCLE_MS units
// Example: To set 1 second with TIMER_CYCLE_MS=10ms, pass duration=100
void setTimerLED_SYS(int duration) {
   timerLED_SYS_counter = duration;
   timerLED_SYS_flag = 0;
}

void setTimer1(int duration) {
   timer1_counter = duration;
   timer1_flag = 0;
}

void setTimer2(int duration) {
   timer2_counter = duration;
   timer2_flag = 0;
}

void setTimer3(int duration) {
   timer3_counter = duration;
   timer3_flag = 0;
}

// Called every TIMER_CYCLE_MS milliseconds
void timerRun() {
   if (timerLED_SYS_counter > 0) {
      timerLED_SYS_counter--;
      if (timerLED_SYS_counter == 0) timerLED_SYS_flag = 1;
   }
   if (timer1_counter > 0) {
      timer1_counter--;
      if (timer1_counter == 0) timer1_flag = 1;
   }
   if (timer2_counter > 0) {
      timer2_counter--;
      if (timer2_counter == 0) timer2_flag = 1;
   }
   if (timer3_counter > 0) {
      timer3_counter--;
      if (timer3_counter == 0) timer3_flag = 1;
   }
}
