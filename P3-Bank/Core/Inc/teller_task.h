/*
 * teller_task.h
 *
 *  Created on: Feb 28, 2023
 *      Author: rickweil
 */

#ifndef INC_TELLER_TASK_H_
#define INC_TELLER_TASK_H_

//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"

#define NUM_TELLERS 3

typedef struct {
    // some parameters to track
    uint8_t instance;
    uint16_t customers_served;
    uint8_t status;

    uint16_t numBreaks;
    uint32_t shortestBreak;
    uint32_t longestBreak;
    uint32_t averageBreak;

    uint8_t onBreak;
    uint8_t breakOverride;
    uint32_t breakTime;
} TELLER_t;

typedef struct {
	uint32_t customers_served;
	uint32_t avg_wait_time;
	uint32_t avg_trans_time;
	uint32_t avg_teller_wait;
	uint32_t max_wait_time;
	uint32_t max_trans_time;
	uint32_t max_teller_wait;
	uint32_t max_queue_depth;
} METRICS_t;

//extern METRICS_t metrics;
extern TELLER_t tellers[NUM_TELLERS];

int teller_task_init(int num);


#endif /* INC_TELLER_TASK_H_ */
