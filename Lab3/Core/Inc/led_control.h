/*
 * led_control.h
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 *  
 *  This file defines LED pin assignments for the traffic light system.
 *  
 *  Crossroad configuration (4-way intersection):
 *  - Road 1 (North-South): PA1 (Red), PA2 (Amber), PA3 (Green)
 *  - Road 2 (East-West):   PA4 (Red), PA5 (Amber), PA6 (Green)
 *  
 *  Opposite sides of the same road share the same LED pins:
 *  - North & South both use PA1, PA2, PA3
 *  - East & West both use PA4, PA5, PA6
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

// ============================================================================
// LED PIN ASSIGNMENTS - Two independent roads
// ============================================================================

// ROAD 1 (North-South direction)
// PA1 - RED, PA2 - AMBER, PA3 - GREEN

// ROAD 2 (East-West direction)
// PA4 - RED, PA5 - AMBER, PA6 - GREEN

// ============================================================================
// LED CONTROL FUNCTIONS
// ============================================================================

// Set all LEDs for Road 1
void set_road1_leds(int red, int amber, int green);

// Set all LEDs for Road 2
void set_road2_leds(int red, int amber, int green);

// Turn off all traffic LEDs
void clear_all_traffic_leds(void);

// Blink all red LEDs
void blink_red_leds(int state);

// Blink all amber LEDs
void blink_amber_leds(int state);

// Blink all green LEDs
void blink_green_leds(int state);

#endif /* INC_LED_CONTROL_H_ */
