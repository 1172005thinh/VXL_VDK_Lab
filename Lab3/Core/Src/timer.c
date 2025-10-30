/*
 * timer.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "main.h"
#include "input_reading.h"
#include "soft_timer.h"
#include "display_7seg.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		timerRun();
		button_reading();
		display_scan();  // Call display scan every 10ms for proper multiplexing
	}
}
