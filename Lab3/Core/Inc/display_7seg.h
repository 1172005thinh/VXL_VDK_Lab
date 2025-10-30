/*
 * display_7seg.h
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 *
 *  Seven-segment display module with multiplexing support
 *  Scanning frequency: 250ms per display (4Hz refresh per digit)
 */

#ifndef INC_DISPLAY_7SEG_H_
#define INC_DISPLAY_7SEG_H_

#include <stdint.h>

// Display buffer for each road (2 digits per road)
extern uint8_t display_buffer_road1[2];  // [tens, ones]
extern uint8_t display_buffer_road2[2];  // [tens, ones]

// Initialize display system
void display_init(void);

// Update display buffers with values (0-99)
void update_display_buffer_road1(uint8_t value);
void update_display_buffer_road2(uint8_t value);

// Scan function - call every 10ms from timer interrupt
// This handles multiplexing of displays
void display_scan(void);

// Directly display a number on a specific road (for debugging)
void display_number_road1(uint8_t number);
void display_number_road2(uint8_t number);

// Display "--" on both roads (error indication)
void display_dash_both_roads(void);

#endif /* INC_DISPLAY_7SEG_H_ */
