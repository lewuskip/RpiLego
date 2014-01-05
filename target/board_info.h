/*
 * board_info.h
 *
 *  Created on: 31 Dec 2013
 *      Author: lewy
 */

#ifndef BOARD_INFO_H_
#define BOARD_INFO_H_

/* ----- 1 ----- */
#define DEBUG_STACK_SIZE_BYTES    2048
#define DEBUG_TASK_PRIORITY       (tskIDLE_PRIORITY + 1)

/* ----- 2 ----- */

/* ----- 3 ----- */
#define MOTOR_STACK_SIZE_BYTES    2048
#define MOTOR_TASK_PRIORITY       (tskIDLE_PRIORITY + 3)

extern struct qik2s9v1_cfg motor_cfg;


#endif /* BOARD_INFO_H_ */
