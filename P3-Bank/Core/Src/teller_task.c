/*
 * teller_task.c
 *
 *  Created on: Feb 28, 2023
 *      Author: rickweil
 */

#include "FreeRTOS.h"
#include "task.h"

#include "teller_task.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"

//TELLER_t tellers[NUM_TELLERS];

// private methods


/**
 * This public method creates a number of teller tasks
 */
int teller_task_init(int num_tellers) {

//    for(int ii=0; ii<num_tellers; ii++) {
//        TELLER_t *t = &tellers[ii];
//        memset(t, 0, sizeof(TELLER_t));
//        t->instance = ii;
//        BaseType_t err = xTaskCreate(teller_task, "TellerTask", 128, &tellers[ii], 12, NULL);
////        assert(err == pdPASS);
//    }

    return 0;
}




