/*
 * input_processing.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "main.h"
#include "input_reading.h"

enum ButtonState {
    BUTTON_RELEASED,
    BUTTON_PRESSED,
    BUTTON_PRESSED_MORE_THAN_1_SECOND
};

static enum ButtonState buttonState = BUTTON_RELEASED;

void fsm_for_input_processing(void) {
    switch (buttonState) {
        case BUTTON_RELEASED:
            if (is_button_pressed(0)) {
                buttonState = BUTTON_PRESSED;
                // TODO: Increase value of PORTA by one unit
                // Example: HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_x);
            }
            break;

        case BUTTON_PRESSED:
            if (!is_button_pressed(0)) {
                buttonState = BUTTON_RELEASED;
            } else if (is_button_pressed_1s(0)) {
                buttonState = BUTTON_PRESSED_MORE_THAN_1_SECOND;
            }
            break;

        case BUTTON_PRESSED_MORE_THAN_1_SECOND:
            if (!is_button_pressed(0)) {
                buttonState = BUTTON_RELEASED;
            } else {
                // TODO: Implement behavior for long press
                // Example: auto-increment a value or trigger a continuous action
            }
            break;
    }
}




