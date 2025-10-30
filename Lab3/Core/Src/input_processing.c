/*
 * input_processing.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "main.h"
#include "input_reading.h"
#include "fsm_traffic3_4.h"

enum ButtonState {
    BUTTON_RELEASED,
    BUTTON_PRESSED,
    BUTTON_PRESSED_MORE_THAN_1_SECOND
};

static enum ButtonState buttonState[3] = {
    BUTTON_RELEASED,
    BUTTON_RELEASED,
    BUTTON_RELEASED
};

void fsm_for_input_processing(void) {
    // Process each button independently
    for (int i = 0; i < 3; i++) {
        switch (buttonState[i]) {
            case BUTTON_RELEASED:
                if (is_button_pressed(i)) {
                    buttonState[i] = BUTTON_PRESSED;
                    // Button action will be handled in fsm_traffic_run()
                }
                break;

            case BUTTON_PRESSED:
                if (!is_button_pressed(i)) {
                    buttonState[i] = BUTTON_RELEASED;
                } else if (is_button_pressed_1s(i)) {
                    buttonState[i] = BUTTON_PRESSED_MORE_THAN_1_SECOND;
                }
                break;

            case BUTTON_PRESSED_MORE_THAN_1_SECOND:
                if (!is_button_pressed(i)) {
                    buttonState[i] = BUTTON_RELEASED;
                } else {
                    // Auto-increment for button 1 (B2_T) when held
                    if (i == 1 && current_state != NORM) {
                        // Auto-increment in modification modes
                    }
                }
                break;
        }
    }
}




