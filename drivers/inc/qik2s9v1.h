/*
 * qik2s9v1.h
 *
 *  Created on: 31 Dec 2013
 *      Author: lewy
 */

#ifndef QIK2S9V1_H_
#define QIK2S9V1_H_

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct qik2s9v1_cfg
{
    const char  *devname;
    uint32_t     baudrate;
}qik2s9v1_cfg;

typedef enum qik2s9v1_num
{
    QIK_MOTOR_0 = 0,
    QIK_MOTOR_1
}qik2s9v1_num;

typedef enum qik2s9v1_coast
{
    QIK_COAST_OFF= 0,
    QIK_COAST_ON
}qik2s9v1_coast;

typedef struct qik2s9v1_mstate
{
    qik2s9v1_num    motor;
    int             speed;
    qik2s9v1_coast  coast;
}qik2s9v1_mstate;

typedef qik2s9v1_mstate qik2s9v1_msg;

int qik2s9v1_init(const struct qik2s9v1_cfg *cfg);
xQueueHandle qik2s9v1_get_queue(void);

#endif /* QIK2S9V1_H_ */
