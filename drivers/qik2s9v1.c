/*
 * qik2s9v1.c
 *
 *  Created on: 31 Dec 2013
 *      Author: lewy
 */

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "debug.h"
#include <board_info.h>
#include "qik2s9v1.h"


#define SAMPLE_PERIOD_MS    100
#define QUEUE_LEN           20
#define QUEUE_WAIT_MS       50

#define NUM_MOTORS          2
// Commands

#define QIK_GET_FIRMWARE_VERSION         0x81
#define QIK_GET_ERROR_BYTE               0x82
#define QIK_GET_CONFIGURATION_PARAMETER  0x83
#define QIK_SET_CONFIGURATION_PARAMETER  0x84

#define QIK_MOTOR_M0_FORWARD             0x88
#define QIK_MOTOR_M0_FORWARD_8_BIT       0x89
#define QIK_MOTOR_M0_REVERSE             0x8A
#define QIK_MOTOR_M0_REVERSE_8_BIT       0x8B
#define QIK_MOTOR_M1_FORWARD             0x8C
#define QIK_MOTOR_M1_FORWARD_8_BIT       0x8D
#define QIK_MOTOR_M1_REVERSE             0x8E
#define QIK_MOTOR_M1_REVERSE_8_BIT       0x8F

// 2s9v1 only
#define QIK_2S9V1_MOTOR_M0_COAST         0x86
#define QIK_2S9V1_MOTOR_M1_COAST         0x87

static xTaskHandle      xHandle = NULL;
static xQueueHandle     xQueue  = NULL;

typedef struct qik2s9v1_dev{
    int fd;
    qik2s9v1_mstate mstate[NUM_MOTORS];
}qik2s9v1_dev;

static struct qik2s9v1_dev *devp = NULL;

static void motorTask(__attribute__((unused)) void *parameters);

/**
 * @brief Allocate a new device
 */
static struct qik2s9v1_dev *qik2s9v1_alloc(void)
{
    struct qik2s9v1_dev *dev;

    dev = (struct qik2s9v1_dev *)pvPortMalloc(sizeof(*dev));
    if (!dev) {
        return NULL;
    }

    return dev;
}

int qik2s9v1_init(const struct qik2s9v1_cfg *cfg)
{
    devp = qik2s9v1_alloc();
    configASSERT( devp );

    // Start main task
    xTaskCreate(motorTask, (signed char *)"Motor Controller", MOTOR_STACK_SIZE_BYTES / 4, NULL, MOTOR_TASK_PRIORITY, &xHandle);
    configASSERT( xHandle );

    xQueue = xQueueCreate( QUEUE_LEN, sizeof(qik2s9v1_msg) );
    configASSERT( xHandle );


    return 0;
}

xQueueHandle qik2s9v1_get_queue(void)
{
    return xQueue;
}

static int set_speed( qik2s9v1_num num, int speed )
{

    return 0;
}

static int set_coast( qik2s9v1_num num, int speed )
{
    return 0;
}

static int decode_msg( qik2s9v1_msg *msg )
{
    if ( msg->coast == QIK_COAST_ON && msg->speed != 0 )
    {
        debug( DEBUG_ERR, DEBUG_MOTOR_DRV, "Can't set cost and speed together \r\n" );
        return -1;
    }

    if ( msg->coast == QIK_COAST_ON )
    {

    }
    else
    {

    }

    return 0;
}
static void motorTask(__attribute__((unused)) void *parameters)
{
    qik2s9v1_msg    msg;

    while(1)
    {
        // Since this module executes at fixed time intervals, we need to
        // block the task until it is time for the next update.
        // The settings field is in ms, to convert to RTOS ticks we need
        // to divide by portTICK_RATE_MS.

        //vTaskDelay(1000 / portTICK_RATE_MS);
        //debug(DEBUG_INFO, DEBUG_MOTOR_DRV, "Tick\r\n");
        if ( pdTRUE == xQueueReceive( xQueue, (void *)&msg, QUEUE_WAIT_MS ) )
        {
            decode_msg( &msg );
            debug( DEBUG_INFO, DEBUG_MOTOR_DRV, "Received motor msg\r\n" );
        }
    }
}
