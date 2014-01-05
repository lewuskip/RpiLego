/*
 * debug.c
 *
 *  Created on: 1 Jan 2014
 *      Author: lewy
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "board_info.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "debug.h"

#define MAX_QUEUE_SIZE  125
#define QUEUE_WAIT_MS   100

// Private variables
static xTaskHandle xHandle = NULL;
static xQueueHandle db_queue = NULL;

typedef struct debug_stat
{
    uint32_t    queue_full;
}debug_stat;

static debug_stat stats =
{
        .queue_full = 0
};

/**
 * Module thread, should not return.
 */
static void debugTask(__attribute__((unused)) void *parameters);

int debug_initialize(void)
{
    // Start main task
    xTaskCreate(debugTask, (signed char *)"Debug", DEBUG_STACK_SIZE_BYTES / 4, NULL, DEBUG_TASK_PRIORITY, &xHandle);
    configASSERT( xHandle );

    /* Use the handle to delete the task. */
    if( xHandle == NULL )
    {
        vTaskDelete( xHandle );
    }

    db_queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(void *));
    configASSERT( db_queue );

    return 0;
}

static const char * debug_msg_type_to_str(debug_type type)
{
    switch (type)
    {
        case DEBUG_INFO:
            return "I";

        case DEBUG_WARN:
            return "W";

        case DEBUG_ERR:
            return "E";

        default:
            return "U";
    }
}

static const char * debug_msg_src_to_str(debug_type type)
{
    switch (type)
    {
        case DEBUG_SYSTEM:
            return "system";

        case DEBUG_MOTOR_DRV:
            return "motor";

        default:
            return "UNK";
    }
}

/**
 * Module thread, should not return.
 */
static void debugTask(__attribute__((unused)) void *parameters)
{
    struct debug_msg *msg = NULL;

    while(true)
    {
        if (xQueueReceive(db_queue, (void *)&msg, QUEUE_WAIT_MS) == errQUEUE_EMPTY) {
            continue;
        }

        if ( msg ) {
            char buff[32];
            time_t rawtime;
            struct tm * timeinfo;
            struct timeval start;
            gettimeofday(&start, NULL);

            time (&rawtime);
            timeinfo = localtime (&rawtime);

            strftime (buff, sizeof(buff), "%T", timeinfo );

            printf("%s:%ld, %s, %s, - %s",   buff,
                                            start.tv_usec / 1000,
                                            debug_msg_type_to_str(msg->type),
                                            debug_msg_src_to_str(msg->source),
                                            msg->buffer);
            vPortFree(msg);
        }
    }
}

int debug(debug_type type, debug_source src, const char * format, ... )
{
    struct debug_msg *msg = NULL;
    va_list args;

    msg = (struct debug_msg *)pvPortMalloc( sizeof(struct debug_msg) );
    configASSERT( msg );

    msg->source = src;
    msg->type = type;

    va_start (args, format);
    vsnprintf (msg->buffer, DEBUG_MAX_MESSAGE_LEN-1,format, args);
    va_end (args);

    if (pdTRUE != xQueueSendToBack(db_queue, (void *)&msg, 0) )
    {
        vPortFree(msg);
        stats.queue_full++;
        return -1;
    }
    return 0;
}
