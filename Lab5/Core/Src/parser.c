/*
 * parser.c
 *
 *  Created on: Nov 14, 2025
 *      Author: HungThinh
 */

#include "parser.h"
#include <string.h>

/* Private variables */
static ParserState parser_state = PARSER_WAIT_START;
static uint8_t command_buffer[10];  // Buffer to store command characters
static uint8_t command_index = 0;   // Current position in command buffer
static uint8_t has_input = 0;       // Set to 1 when non-whitespace input detected

/* Public variables */
uint8_t command_flag = 0;   // Set to 1 when a valid command is received
uint8_t command_data = 0;   // Stores the command type
uint8_t command_error_flag = 0;  // Set to 1 when invalid command is entered

/**
 * @brief Initialize the command parser FSM
 */
void parser_init(void) {
    parser_state = PARSER_WAIT_START;
    command_index = 0;
    command_flag = 0;
    command_data = 0;
    has_input = 0;
    command_error_flag = 0;
    memset(command_buffer, 0, sizeof(command_buffer));
}

/**
 * @brief Command parser FSM - processes one character at a time
 * @param c Character received from UART
 * 
 * FSM States:
 * - PARSER_WAIT_START: Waits for '!' to indicate start of command
 * - PARSER_READ_COMMAND: Reads command characters until '#' is received
 * 
 * Valid commands:
 * - !RST# : Request sensor data (sets command_data = COMMAND_RST)
 * - !OK#  : Acknowledge reception (sets command_data = COMMAND_OK)
 * 
 * Commands execute immediately when '#' is received (no ENTER required)
 */
void command_parser_fsm(uint8_t c) {
    // Handle ENTER key - check for malformed input
    if (c == '\r' || c == '\n') {
        if (has_input) {
            // Non-whitespace input without proper !CMD# format
            command_error_flag = 1;
            has_input = 0;
        }
        return;
    }
    
    switch (parser_state) {
        case PARSER_WAIT_START:
            // Wait for start character '!'
            if (c == '!') {
                parser_state = PARSER_READ_COMMAND;
                command_index = 0;
                has_input = 0;
                memset(command_buffer, 0, sizeof(command_buffer));
            } else if (c != ' ' && c != '\t') {
                // Non-whitespace character without '!' start - malformed input
                has_input = 1;
            }
            break;

        case PARSER_READ_COMMAND:
            // Check for end character '#'
            if (c == '#') {
                // Null-terminate the command string
                command_buffer[command_index] = '\0';
                
                // Check which command was received and execute immediately
                if (strcmp((char*)command_buffer, "RST") == 0) {
                    command_data = COMMAND_RST;
                    command_flag = 1;
                } else if (strcmp((char*)command_buffer, "OK") == 0) {
                    command_data = COMMAND_OK;
                    command_flag = 1;
                } else if (command_index > 0) {
                    // Invalid command detected (has content but not RST or OK)
                    command_error_flag = 1;
                }
                
                // Reset to wait for next command
                parser_state = PARSER_WAIT_START;
                command_index = 0;
                has_input = 0;
            } 
            // Check for unexpected start character (restart parsing)
            else if (c == '!') {
                command_index = 0;
                has_input = 0;
                memset(command_buffer, 0, sizeof(command_buffer));
            }
            // Store command character
            else if (command_index < sizeof(command_buffer) - 1) {
                command_buffer[command_index++] = c;
            } else {
                // Buffer overflow, reset parser
                parser_state = PARSER_WAIT_START;
                command_index = 0;
                has_input = 0;
            }
            break;

        case PARSER_WAIT_END:
            // This state is not used in current implementation
            // (merged with PARSER_READ_COMMAND for simplicity)
            break;

        default:
            // Invalid state, reset
            parser_state = PARSER_WAIT_START;
            command_index = 0;
            break;
    }
}

/**
 * @brief Check if a command is available
 * @return 1 if command_flag is set, 0 otherwise
 */
uint8_t is_command_available(void) {
    return command_flag;
}

/**
 * @brief Get the parsed command
 * @return The command type (COMMAND_RST or COMMAND_OK)
 */
uint8_t get_command(void) {
    return command_data;
}

/**
 * @brief Clear the command flag after processing
 */
void clear_command_flag(void) {
    command_flag = 0;
    command_data = 0;
}

/**
 * @brief Check if an error occurred (invalid command)
 * @return 1 if error flag is set, 0 otherwise
 */
uint8_t is_command_error(void) {
    return command_error_flag;
}

/**
 * @brief Clear the error flag after handling
 */
void clear_command_error(void) {
    command_error_flag = 0;
}


