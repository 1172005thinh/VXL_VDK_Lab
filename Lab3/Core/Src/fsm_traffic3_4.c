/*
 * fsm_traffic3_4.c
 *
 *  Created on: Oct 30, 2025
 *      Author: HungThinh
 */

#include "fsm_traffic3_4.h"
#include "main.h"
#include "stm32f103c6.h"
#include "soft_timer.h"
#include "input_reading.h"
#include "led_control.h"
#include "display_7seg.h"

// Default durations (in seconds)
int red_duration = 5;
int amber_duration = 2;
int green_duration = 3;

// Demo value for modification modes (01-99)
int demo_value = 1;

// Current state
TrafficState current_state = INIT;

// Traffic light sub-state for normal mode
typedef enum {
    ROAD1_RED_ROAD2_GREEN,
    ROAD1_RED_ROAD2_AMBER,
    ROAD1_GREEN_ROAD2_RED,
    ROAD1_AMBER_ROAD2_RED
} TrafficPhase;

static TrafficPhase traffic_phase = ROAD1_RED_ROAD2_GREEN;
static int traffic_counter = 0;

// LED blink timer for modification modes (2Hz = 500ms period, 250ms on/off)
static int blink_counter = 0;
static int blink_state = 0;

void fsm_traffic_init(void) {
    current_state = NORM;
    traffic_phase = ROAD1_RED_ROAD2_GREEN;
    traffic_counter = green_duration;
    blink_counter = 0;
    demo_value = 1;
}

void fsm_traffic_run(void) {
    switch (current_state) {
        case INIT:
            current_state = NORM;
            break;

        case NORM:
            if (red_duration != (green_duration + amber_duration)) {
                if (timer2_flag == 1) {
                    setTimer2(250 / TIMER_CYCLE_MS);
                    blink_state = !blink_state;
                    blink_amber_leds(blink_state);
                }
                display_dash_both_roads();
            } else {
                run_normal_traffic();
            }
            
            if (is_button_pressed_short(0)) {
                clear_all_traffic_leds();
                current_state = RED;
                demo_value = red_duration;
                blink_counter = 0;
            }
            break;

        case RED:
            display_mode(2);
            display_duration(demo_value);
            blink_leds_by_mode();
            
            if (is_button_pressed_short(0)) {
                clear_all_traffic_leds();
                current_state = AMBER;
                demo_value = amber_duration;
                blink_counter = 0;
            } else if (is_button_pressed_short(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_1s(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_short(2)) {
                red_duration = demo_value;
            }
            break;

        case AMBER:
            display_mode(3);
            display_duration(demo_value);
            blink_leds_by_mode();
            
            if (is_button_pressed_short(0)) {
                clear_all_traffic_leds();
                current_state = GREEN;
                demo_value = green_duration;
                blink_counter = 0;
            } else if (is_button_pressed_short(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_1s(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_short(2)) {
                amber_duration = demo_value;
            }
            break;

        case GREEN:
            display_mode(4);
            display_duration(demo_value);
            blink_leds_by_mode();
            
            if (is_button_pressed_short(0)) {
                clear_all_traffic_leds();
                current_state = NORM;
                traffic_phase = ROAD1_RED_ROAD2_GREEN;
                traffic_counter = green_duration;
            } else if (is_button_pressed_short(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_1s(1)) {
                demo_value++;
                if (demo_value > 99) demo_value = 1;
            } else if (is_button_pressed_short(2)) {
                green_duration = demo_value;
            }
            break;
    }
}

void run_normal_traffic(void) {
    if (timer1_flag == 1) {
        setTimer1(1000 / TIMER_CYCLE_MS);
        
        if (traffic_counter > 0) {
            traffic_counter--;
        }
        
        if (traffic_counter == 0) {
            switch (traffic_phase) {
                case ROAD1_RED_ROAD2_GREEN:
                    traffic_phase = ROAD1_RED_ROAD2_AMBER;
                    traffic_counter = amber_duration;
                    break;
                    
                case ROAD1_RED_ROAD2_AMBER:
                    traffic_phase = ROAD1_GREEN_ROAD2_RED;
                    traffic_counter = green_duration;
                    break;
                    
                case ROAD1_GREEN_ROAD2_RED:
                    traffic_phase = ROAD1_AMBER_ROAD2_RED;
                    traffic_counter = amber_duration;
                    break;
                    
                case ROAD1_AMBER_ROAD2_RED:
                    traffic_phase = ROAD1_RED_ROAD2_GREEN;
                    traffic_counter = green_duration;
                    break;
            }
        }
        
        switch (traffic_phase) {
            case ROAD1_RED_ROAD2_GREEN:
                set_road1_leds(1, 0, 0);
                set_road2_leds(0, 0, 1);
                break;
                
            case ROAD1_RED_ROAD2_AMBER:
                set_road1_leds(1, 0, 0);
                set_road2_leds(0, 1, 0);
                break;
                
            case ROAD1_GREEN_ROAD2_RED:
                set_road1_leds(0, 0, 1);
                set_road2_leds(1, 0, 0);
                break;
                
            case ROAD1_AMBER_ROAD2_RED:
                set_road1_leds(0, 1, 0);
                set_road2_leds(1, 0, 0);
                break;
        }
    }
    
    int road1_time = 0;
    int road2_time = 0;
    
    switch (traffic_phase) {
        case ROAD1_RED_ROAD2_GREEN:
            road1_time = traffic_counter + amber_duration;
            road2_time = traffic_counter;
            break;
            
        case ROAD1_RED_ROAD2_AMBER:
            road1_time = traffic_counter;
            road2_time = traffic_counter;
            break;
            
        case ROAD1_GREEN_ROAD2_RED:
            road1_time = traffic_counter;
            road2_time = traffic_counter + amber_duration;
            break;
            
        case ROAD1_AMBER_ROAD2_RED:
            road1_time = traffic_counter;
            road2_time = traffic_counter;
            break;
    }
    
    update_display_buffer_road1(road1_time);
    update_display_buffer_road2(road2_time);
}

void display_mode(int mode) {
    update_display_buffer_road1(mode);
}

void display_duration(int value) {
    update_display_buffer_road2(value);
}

void blink_leds_by_mode(void) {
    if (timer2_flag == 1) {
        setTimer2(250 / TIMER_CYCLE_MS);
        blink_state = !blink_state;
        
        if (current_state == RED) {
            blink_red_leds(blink_state);
        } else if (current_state == AMBER) {
            blink_amber_leds(blink_state);
        } else if (current_state == GREEN) {
            blink_green_leds(blink_state);
        }
    }
}
