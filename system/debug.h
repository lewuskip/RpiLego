/*
 * debug.h
 *
 *  Created on: 1 Jan 2014
 *      Author: lewy
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#define DEBUG_MAX_MESSAGE_LEN   120

typedef enum debug_type
{
    DEBUG_INFO = 0,
    DEBUG_WARN,
    DEBUG_ERR
}debug_type;

typedef enum debug_source {
    DEBUG_SYSTEM,
    DEBUG_MOTOR_DRV,
}debug_source;

typedef struct debug_msg
{
    debug_source    source;
    debug_type      type;
    char            buffer[DEBUG_MAX_MESSAGE_LEN];
}debug_msg;

int debug_initialize(void);
int debug(debug_type type, debug_source src, const char * format, ... );


#endif /* DEBUG_H_ */
