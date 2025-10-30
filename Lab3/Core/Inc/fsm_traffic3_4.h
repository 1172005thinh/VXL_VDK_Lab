/*
 * fsm_traffic3_4.h
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#ifndef INC_FSM_TRAFFIC3_4_H_
#define INC_FSM_TRAFFIC3_4_H_

// FSM States for Traffic Light System
typedef enum {
    INIT,
    NORM,     // Normal mode - traffic light running
    RED,      // Modify RED LED duration mode
    AMBER,    // Modify AMBER LED duration mode
    GREEN     // Modify GREEN LED duration mode
} TrafficState;

// External variables for LED durations (in seconds)
extern int red_duration;
extern int amber_duration;
extern int green_duration;

// Demo value for modification (01-99)
extern int demo_value;

// Current mode/state
extern TrafficState current_state;

// Initialize the FSM
void fsm_traffic_init(void);

// Main FSM function - call in main loop
void fsm_traffic_run(void);

// Normal traffic light sequence
void run_normal_traffic(void);

// Display mode on seven-segment
void display_mode(int mode);

// Display duration value on seven-segment
void display_duration(int value);

// Blink LEDs based on current mode
void blink_leds_by_mode(void);

#endif /* INC_FSM_TRAFFIC3_4_H_ */
