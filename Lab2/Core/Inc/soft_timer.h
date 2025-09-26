/*
 * soft_timer.h
 *
 *  Created on: Sep 24, 2025
 *      Author: HungThinh
 */

 #ifndef INC_SOFT_TIMER_H_
 #define INC_SOFT_TIMER_H_

 #include <stdbool.h>

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
