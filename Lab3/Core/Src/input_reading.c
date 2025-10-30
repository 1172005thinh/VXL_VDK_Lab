/*
 * input_reading.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "main.h"

// We aim to work with more than one button
#define NO_OF_BUTTONS 3
// Button 0: Mode selection (B1_T)
// Button 1: Increment value (B2_T)
// Button 2: Set value (B3_T)

// Timer interrupt duration is 10 ms, so to pass 1 second,
// we need to jump to the interrupt service routine 100 times
#define DURATION_FOR_AUTO_INCREASING 100

#define BUTTON_IS_PRESSED   GPIO_PIN_RESET
#define BUTTON_IS_RELEASED  GPIO_PIN_SET

// The buffer where the final result is stored after debouncing
static GPIO_PinState buttonBuffer[NO_OF_BUTTONS];

// We define two buffers for debouncing
static GPIO_PinState debounceButtonBuffer1[NO_OF_BUTTONS];
static GPIO_PinState debounceButtonBuffer2[NO_OF_BUTTONS];

// We define a flag for a button pressed more than 1 second
static uint8_t flagForButtonPress1s[NO_OF_BUTTONS];

// We define a counter for automatically increasing the value
// after the button is pressed more than 1 second
static uint16_t counterForButtonPress1s[NO_OF_BUTTONS];

// Flag for detecting single button press (edge-triggered, not level-triggered)
static uint8_t flagForButtonPressShort[NO_OF_BUTTONS];

void button_reading(void) {
    for (char i = 0; i < NO_OF_BUTTONS; i++) {
        debounceButtonBuffer2[i] = debounceButtonBuffer1[i];
        
        // Read different button pins based on index
        if (i == 0) {
            debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(_14_GPIO_Port, _14_Pin);
        } else if (i == 1) {
            debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(_15_GPIO_Port, _15_Pin);
        } else if (i == 2) {
            debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(_16_GPIO_Port, _16_Pin);
        }

        if (debounceButtonBuffer1[i] == debounceButtonBuffer2[i]) {
            // If both buffers match, update the main button buffer
            GPIO_PinState previousState = buttonBuffer[i];
            buttonBuffer[i] = debounceButtonBuffer1[i];
            
            // Detect button press edge (transition from released to pressed)
            if (previousState == BUTTON_IS_RELEASED && buttonBuffer[i] == BUTTON_IS_PRESSED) {
                flagForButtonPressShort[i] = 1;
            }
        }

        if (buttonBuffer[i] == BUTTON_IS_PRESSED) {
            // If a button is pressed, we start counting
            if (counterForButtonPress1s[i] < DURATION_FOR_AUTO_INCREASING) {
                counterForButtonPress1s[i]++;
            } else {
                // The flag is turned on when 1 second has passed
                // since the button is pressed.
                flagForButtonPress1s[i] = 1;
                // TODO: Add handling for long-press action
            }
        } else {
            counterForButtonPress1s[i] = 0;
            flagForButtonPress1s[i] = 0;
        }
    }
}

unsigned char is_button_pressed(uint8_t index) {
    if (index >= NO_OF_BUTTONS) return 0;
    return (buttonBuffer[index] == BUTTON_IS_PRESSED);
}

unsigned char is_button_pressed_1s(uint8_t index) {
    if (index >= NO_OF_BUTTONS) return 0xFF;
    return (flagForButtonPress1s[index] == 1);
}

unsigned char is_button_pressed_short(uint8_t index) {
    if (index >= NO_OF_BUTTONS) return 0;
    if (flagForButtonPressShort[index] == 1) {
        flagForButtonPressShort[index] = 0;  // Clear flag after reading (one-shot)
        return 1;
    }
    return 0;
}
