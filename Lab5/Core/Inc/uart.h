/*
 * uart.h
 *
 *  Created on: Nov 14, 2025
 *      Author: HungThinh
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* FSM states for UART communication */
typedef enum {
    UART_IDLE,           // Idle state, waiting for RST command
    UART_SEND_DATA,      // Send ADC data packet to terminal
    UART_WAIT_ACK,       // Wait for OK acknowledgment from terminal
    UART_RESEND_DATA     // Timeout occurred, resend data packet
} UartCommState;

/* Function prototypes */

/**
 * @brief Initialize the UART communication FSM
 * @details Resets FSM state to IDLE
 */
void uart_comm_init(void);

/**
 * @brief UART communication FSM
 * @param huart UART handle
 * @param adc_value Current ADC reading to send
 * 
 * @details This FSM implements the communication protocol:
 *   1. UART_IDLE: Wait for COMMAND_RST from parser
 *   2. UART_SEND_DATA: Send !ADC=xxxx# packet to terminal
 *   3. UART_WAIT_ACK: Wait for COMMAND_OK (3-second timeout)
 *   4. UART_RESEND_DATA: Timeout occurred, resend same packet
 * 
 * The FSM checks for command_flag from parser module and handles
 * the timeout by counting HAL_GetTick() milliseconds.
 */
void uart_communication_fsm(UART_HandleTypeDef *huart, uint32_t adc_value);

/**
 * @brief Get current communication state
 * @return Current FSM state
 */
UartCommState get_uart_state(void);

/**
 * @brief Format and send ADC data packet
 * @param huart UART handle
 * @param adc_value ADC value to send
 * @details Sends packet in format: !ADC=xxxx#
 */
void send_adc_packet(UART_HandleTypeDef *huart, uint32_t adc_value);

#endif /* INC_UART_H_ */
