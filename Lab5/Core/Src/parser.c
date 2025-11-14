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

/* Public variables */
uint8_t command_flag = 0;   // Set to 1 when a valid command is received
uint8_t command_data = 0;   // Stores the command type

/**
 * @brief Initialize the command parser FSM
 */
void parser_init(void) {
    parser_state = PARSER_WAIT_START;
    command_index = 0;
    command_flag = 0;
    command_data = 0;
    memset(command_buffer, 0, sizeof(command_buffer));
}

/**
 * @brief Command parser FSM - processes one character at a time
 * @param c Character received from UART
 * 
 * FSM States:
 * - PARSER_WAIT_START: Waits for '!' to indicate start of command
 * - PARSER_READ_COMMAND: Reads command characters (RST or OK)
 * - PARSER_WAIT_END: Waits for '#' to indicate end of command
 * 
 * Valid commands:
 * - !RST# : Request sensor data (sets command_data = COMMAND_RST)
 * - !OK#  : Acknowledge reception (sets command_data = COMMAND_OK)
 */
void command_parser_fsm(uint8_t c) {
    switch (parser_state) {
        case PARSER_WAIT_START:
            // Wait for start character '!'
            if (c == '!') {
                parser_state = PARSER_READ_COMMAND;
                command_index = 0;
                memset(command_buffer, 0, sizeof(command_buffer));
            }
            break;

        case PARSER_READ_COMMAND:
            // Check for end character '#'
            if (c == '#') {
                // Null-terminate the command string
                command_buffer[command_index] = '\0';
                
                // Check which command was received
                if (strcmp((char*)command_buffer, "RST") == 0) {
                    command_data = COMMAND_RST;
                    command_flag = 1;
                } else if (strcmp((char*)command_buffer, "OK") == 0) {
                    command_data = COMMAND_OK;
                    command_flag = 1;
                }
                // Invalid command, just reset
                
                // Reset to wait for next command
                parser_state = PARSER_WAIT_START;
                command_index = 0;
            } 
            // Check for unexpected start character (restart parsing)
            else if (c == '!') {
                command_index = 0;
                memset(command_buffer, 0, sizeof(command_buffer));
            }
            // Store command character
            else if (command_index < sizeof(command_buffer) - 1) {
                command_buffer[command_index++] = c;
            } else {
                // Buffer overflow, reset parser
                parser_state = PARSER_WAIT_START;
                command_index = 0;
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


