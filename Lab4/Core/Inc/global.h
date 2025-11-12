/*
 * global.h
 *
 *  Created on: Nov 12, 2025
 *      Author: HungThinh
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include <stdint.h>

// Error codes for the scheduler
typedef enum {
    ERROR_SCH_OK = 0,
    ERROR_SCH_TOO_MANY_TASKS,
    ERROR_SCH_CANNOT_DELETE_TASK,
    ERROR_SCH_WAITING_FOR_SLAVE_TO_ACK,
    ERROR_SCH_WAITING_FOR_START_COMMAND_FROM_MASTER,
    ERROR_SCH_ONE_OR_MORE_SLAVES_DID_NOT_START,
    ERROR_SCH_LOST_SLAVE,
    ERROR_SCH_CAN_BUS_ERROR,
    ERROR_I2C_WRITE_BYTE_AT24C64,
    ERROR_TICK_OVERFLOW
} ERROR_CODE_t;

// Global error code variable
extern ERROR_CODE_t Error_code_G;

#endif /* INC_GLOBAL_H_ */
