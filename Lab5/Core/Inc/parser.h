/*
 * parser.h
 *
 *  Created on: Nov 14, 2025
 *      Author: HungThinh
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include <stdint.h>

/* Command definitions */
#define COMMAND_RST 1
#define COMMAND_OK 2

/* FSM states for command parser */
typedef enum {
    PARSER_WAIT_START,     // Waiting for '!' character
    PARSER_READ_COMMAND,   // Reading command characters
    PARSER_WAIT_END        // Waiting for '#' character
} ParserState;

/* Global variables */
extern uint8_t command_flag;       // Flag indicating a command was received
extern uint8_t command_data;       // The command type (COMMAND_RST or COMMAND_OK)
extern uint8_t command_error_flag; // Flag indicating an invalid command was entered

/* Function prototypes */

/**
 * @brief Initialize the command parser FSM
 * @details Resets all parser states and variables
 */
void parser_init(void);

/**
 * @brief Command parser FSM - processes one character from buffer
 * @param c The character to process
 * @details This function implements a state machine that:
 *          - PARSER_WAIT_START: waits for '!' to start command
 *          - PARSER_READ_COMMAND: reads command characters (R,S,T or O,K)
 *          - PARSER_WAIT_END: waits for '#' to complete command
 *          When a valid command is detected, sets command_flag and command_data
 */
void command_parser_fsm(uint8_t c);

/**
 * @brief Check if a command is available
 * @return 1 if a command has been parsed, 0 otherwise
 */
uint8_t is_command_available(void);

/**
 * @brief Get the parsed command
 * @return The command type (COMMAND_RST or COMMAND_OK)
 */
uint8_t get_command(void);

/**
 * @brief Clear the command flag after processing
 */
void clear_command_flag(void);

/**
 * @brief Check if an error occurred (invalid command)
 * @return 1 if error flag is set, 0 otherwise
 */
uint8_t is_command_error(void);

/**
 * @brief Clear the error flag after handling
 */
void clear_command_error(void);

#endif /* INC_PARSER_H_ */
