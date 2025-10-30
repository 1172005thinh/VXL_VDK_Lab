/*
 * display_7seg.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "display_7seg.h"
#include "main.h"

// Display buffers
uint8_t display_buffer_road1[2] = {0, 0};  // [tens, ones]
uint8_t display_buffer_road2[2] = {0, 0};  // [tens, ones]

// Multiplexing state
static uint8_t current_digit = 0;  // 0 or 1 (which digit is being displayed)
static uint8_t scan_counter = 0;   // Counter for scanning timing

// Seven-segment encoding for COMMON ANODE (active high)
// Segments: a b c d e f g (bit 6 to bit 0)
// 1 = segment OFF, 0 = segment ON
static const uint8_t seg_patterns[13] = {
    //abcdefg
    0b0000001, //0
    0b1001111, //1
    0b0010010, //2
    0b0000110, //3
    0b1001100, //4
    0b0100100, //5
    0b0100000, //6
    0b0001111, //7
    0b0000000, //8
    0b0000100, //9
    0b1111111, //10 - OFF (all segments off)
    0b0110000, //11 - E
    0b1111110  //12 - '-'
};


void display_init(void) {
    display_buffer_road1[0] = 0;
    display_buffer_road1[1] = 0;
    display_buffer_road2[0] = 0;
    display_buffer_road2[1] = 0;
    current_digit = 0;
    scan_counter = 0;
    
    // Disable all displays initially
    // For COMMON ANODE with PNP transistors: HIGH = OFF, LOW = ON
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // EN_SEG_1_ROAD1 OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // EN_SEG_2_ROAD1 OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // EN_SEG_1_ROAD2 OFF
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // EN_SEG_2_ROAD2 OFF
}

void update_display_buffer_road1(uint8_t value) {
    if (value > 99) value = 1;
    display_buffer_road1[0] = value / 10;  // Tens digit
    display_buffer_road1[1] = value % 10;  // Ones digit
}

void update_display_buffer_road2(uint8_t value) {
    if (value > 99) value = 1;
    display_buffer_road2[0] = value / 10;  // Tens digit
    display_buffer_road2[1] = value % 10;  // Ones digit
}

static void write_segment_road1(uint8_t pattern) {
    // Road 1 segments: PA7-PA12, PB3 (a-g)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (pattern & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // a
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (pattern & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // b
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (pattern & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // c
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (pattern & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // d
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (pattern & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // e
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (pattern & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // f
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (pattern & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // g
}

static void write_segment_road2(uint8_t pattern) {
    // Road 2 segments: PB6-PB12 (a-g)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (pattern & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // a
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (pattern & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // b
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (pattern & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // c
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (pattern & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // d
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (pattern & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // e
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (pattern & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // f
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (pattern & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // g
}

void display_scan(void) {
    // Scan every 25 x 10ms = 250ms per digit (4Hz per digit, 2Hz complete refresh)
    scan_counter++;
    if (scan_counter >= 25) {
        scan_counter = 0;
        
        // Turn off all enable pins first
        // For COMMON ANODE with PNP: HIGH = OFF (PNP not conducting)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // EN_SEG_1_ROAD1 OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // EN_SEG_2_ROAD1 OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // EN_SEG_1_ROAD2 OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // EN_SEG_2_ROAD2 OFF
        
        // Update segments for Road 1
        uint8_t digit_road1 = display_buffer_road1[current_digit];
        if (digit_road1 < 13) {
            write_segment_road1(seg_patterns[digit_road1]);
        }
        
        // Update segments for Road 2
        uint8_t digit_road2 = display_buffer_road2[current_digit];
        if (digit_road2 < 13) {
            write_segment_road2(seg_patterns[digit_road2]);
        }
        
        // Enable the appropriate digit
        // For COMMON ANODE with PNP: LOW = ON (PNP conducting, provides +5V to common anode)
        if (current_digit == 0) {
            // Display tens digit
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // EN_SEG_1_ROAD1 ON
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // EN_SEG_1_ROAD2 ON
        } else {
            // Display ones digit
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);   // EN_SEG_2_ROAD1 ON
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // EN_SEG_2_ROAD2 ON
        }
        
        // Toggle to next digit
        current_digit = 1 - current_digit;
    }
}

void display_number_road1(uint8_t number) {
    update_display_buffer_road1(number);
}

void display_number_road2(uint8_t number) {
    update_display_buffer_road2(number);
}

void display_dash_both_roads(void) {
    display_buffer_road1[0] = 12;  // tens digit = dash
    display_buffer_road1[1] = 12;  // ones digit = dash
    display_buffer_road2[0] = 12;  // tens digit = dash
    display_buffer_road2[1] = 12;  // ones digit = dash
}
