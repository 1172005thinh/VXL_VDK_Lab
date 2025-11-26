/*
 * uart.c
 *
 *  Created on: Nov 14, 2025
 *      Author: HungThinh
 */

#include "uart.h"
#include "parser.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
static UartCommState uart_state = UART_IDLE;
static uint32_t timeout_start = 0;       // Timestamp when we started waiting for ACK
static uint32_t last_adc_value = 0;      // Store last ADC value for retransmission
static const uint32_t TIMEOUT_MS = 3000; // 3-second timeout

/**
 * @brief Initialize the UART communication FSM
 */
void uart_comm_init(void) {
    uart_state = UART_IDLE;
    timeout_start = 0;
    last_adc_value = 0;
}

/**
 * @brief Get current communication state
 * @return Current FSM state
 */
UartCommState get_uart_state(void) {
    return uart_state;
}

/**
 * @brief Format and send ADC data packet
 * @param huart UART handle pointer
 * @param adc_value ADC value to send (0-4095)
 * 
 * Sends packet in format: !ADC=xxxx#
 * Example: !ADC=2048# for half-scale reading
 */
void send_adc_packet(UART_HandleTypeDef *huart, uint32_t adc_value) {
    char tx_buffer[30];
    int len = sprintf(tx_buffer, "!ADC=%lu#\r\n", adc_value);
    HAL_UART_Transmit(huart, (uint8_t*)tx_buffer, len, 1000);
}

/**
 * @brief UART communication FSM
 * @param huart UART handle pointer
 * @param adc_value Current ADC reading
 * 
 * FSM Operation:
 * 
 * UART_IDLE:
 *   - Wait for COMMAND_RST from parser
 *   - When received, transition to UART_SEND_DATA
 * 
 * UART_SEND_DATA:
 *   - Format and send !ADC=xxxx# packet
 *   - Store current ADC value for potential retransmission
 *   - Start timeout timer
 *   - Transition to UART_WAIT_ACK
 * 
 * UART_WAIT_ACK:
 *   - Wait for COMMAND_OK from parser
 *   - If OK received, clear command and return to UART_IDLE
 *   - If timeout (3 seconds) expires, transition to UART_RESEND_DATA
 * 
 * UART_RESEND_DATA:
 *   - Resend the same ADC packet (using stored value)
 *   - Restart timeout timer
 *   - Transition back to UART_WAIT_ACK
 */
void uart_communication_fsm(UART_HandleTypeDef *huart, uint32_t adc_value) {
    uint32_t current_time = HAL_GetTick();
    
    switch (uart_state) {
        case UART_IDLE:
            // Wait for RST command from terminal
            if (is_command_available() && get_command() == COMMAND_RST) {
                // Clear the command flag
                clear_command_flag();
                
                // Transition to send data state
                uart_state = UART_SEND_DATA;
            }
            break;

        case UART_SEND_DATA:
            // Send ADC data packet to terminal
            send_adc_packet(huart, adc_value);
            
            // Store ADC value for potential retransmission
            last_adc_value = adc_value;
            
            // Start timeout timer
            timeout_start = current_time;
            
            // Transition to wait for ACK
            uart_state = UART_WAIT_ACK;
            break;

        case UART_WAIT_ACK:
            // Check if OK command received
            if (is_command_available() && get_command() == COMMAND_OK) {
                // ACK received successfully
                clear_command_flag();
                
                // Send exit confirmation message
                const char* exit_msg = "[CMD] Exit\r\n";
                HAL_UART_Transmit(huart, (uint8_t*)exit_msg, 12, 100);
                
                // Return to idle state
                uart_state = UART_IDLE;
            }
            // Check if timeout occurred (3 seconds)
            else if ((current_time - timeout_start) >= TIMEOUT_MS) {
                // Timeout occurred, need to resend
                uart_state = UART_RESEND_DATA;
            }
            // Also handle case where RST command is received again (restart)
            else if (is_command_available() && get_command() == COMMAND_RST) {
                clear_command_flag();
                uart_state = UART_SEND_DATA;
            }
            break;

        case UART_RESEND_DATA:
            // Resend the same ADC packet (use stored value)
            send_adc_packet(huart, last_adc_value);
            
            // Restart timeout timer
            timeout_start = current_time;
            
            // Go back to waiting for ACK
            uart_state = UART_WAIT_ACK;
            break;

        default:
            // Invalid state, reset to IDLE
            uart_state = UART_IDLE;
            break;
    }
}


