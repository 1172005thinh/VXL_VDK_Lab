/*
 * led_control.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "led_control.h"
#include "main.h"

void set_road1_leds(int red, int amber, int green) {
    // Control Road 1 (North-South) LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, red ? GPIO_PIN_SET : GPIO_PIN_RESET);      // RED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, amber ? GPIO_PIN_SET : GPIO_PIN_RESET);    // AMBER
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, green ? GPIO_PIN_SET : GPIO_PIN_RESET);    // GREEN
}

void set_road2_leds(int red, int amber, int green) {
    // Control Road 2 (East-West) LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, red ? GPIO_PIN_SET : GPIO_PIN_RESET);      // RED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, amber ? GPIO_PIN_SET : GPIO_PIN_RESET);    // AMBER
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, green ? GPIO_PIN_SET : GPIO_PIN_RESET);    // GREEN
}

void clear_all_traffic_leds(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // RED Road 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  // AMBER Road 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // GREEN Road 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // RED Road 2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // AMBER Road 2
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // GREEN Road 2
}

void blink_red_leds(int state) {
    // Blink both road's red LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void blink_amber_leds(int state) {
    // Blink both road's amber LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void blink_green_leds(int state) {
    // Blink both road's green LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
