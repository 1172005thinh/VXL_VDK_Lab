/*
 * input_reading.h
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#ifndef INC_INPUT_READING_H_
#define INC_INPUT_READING_H_

void button_reading(void);
unsigned char is_button_pressed(unsigned char index);
unsigned char is_button_pressed_1s(unsigned char index);

// New function to check for single button press (edge-triggered)
unsigned char is_button_pressed_short(unsigned char index);

#endif /* INC_INPUT_READING_H_ */
