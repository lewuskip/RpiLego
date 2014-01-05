#include <stdio.h>
#include <unistd.h>

#include "board_info.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "qik2s9v1.h"

int initilaize_drivers(void)
{
    int err = 0;
    /* Motor Driver */
    if ( 0 != (err = qik2s9v1_init(&motor_cfg) ) )
    {
        return err;
    }
    return 0;
}

static xTaskHandle      xHandle = NULL;
static xQueueHandle     xQueue  = NULL;

static void testTask(__attribute__((unused)) void *parameters)
{
    portTickType    lastSysTime;
    qik2s9v1_msg    msg;

    xQueue = qik2s9v1_get_queue();
    configASSERT( xQueue );

    // Main task loop
    lastSysTime = xTaskGetTickCount();

    msg.speed = 100;
    msg.motor = QIK_MOTOR_0;
    msg.coast  = QIK_COAST_OFF;
    xQueueSendToBack( xQueue, (void *)&msg, 0 );
    vTaskDelay( 1000 / portTICK_RATE_MS );

    msg.speed = -100;
    msg.motor = QIK_MOTOR_0;
    msg.coast  = QIK_COAST_OFF;
    xQueueSendToBack( xQueue, (void *)&msg, 0 );
    vTaskDelay( 1000 / portTICK_RATE_MS );

    msg.speed = 0;
    msg.motor = QIK_MOTOR_0;
    msg.coast  = QIK_COAST_ON;
    xQueueSendToBack( xQueue, (void *)&msg, 0 );
    vTaskDelay( 1000 / portTICK_RATE_MS );

    msg.coast  = QIK_COAST_OFF;

    while(1)
    {
        xQueueSendToBack( xQueue, (void *)&msg, 10 / portTICK_RATE_MS );

        vTaskDelayUntil(&lastSysTime, 1000 / portTICK_RATE_MS);

        msg.speed += 10;
        if ( msg.speed >= 255 ) {
            msg.speed = 0;
        }

        debug(DEBUG_INFO, DEBUG_MOTOR_DRV, "Test motor \r\n");
    }
}

int main() {

    debug_initialize();

    debug(DEBUG_INFO, DEBUG_SYSTEM, "Start \r\n");

    if ( 0 != initilaize_drivers() ) goto exit;
    debug(DEBUG_INFO, DEBUG_SYSTEM, "Drivers initialised \r\n");

    // Start test task
    xTaskCreate(testTask, (signed char *)"Test motor", 2048 / 4, NULL, 3, &xHandle);
    configASSERT( xHandle );

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

exit:
    printf("Exiting\r\n");
    return 0;
}
