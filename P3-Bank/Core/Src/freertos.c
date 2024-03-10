/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "teller_task.h"
#include "rng.h"
#include "usart.h"
#include "MFS.h"
#include <stdio.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONEMIN			100u
#define ONEHOUR			6000u
#define ONEDAY			42000u
//#define ONEDAY			2000u
#define THIRTYSEC		50u
#define FOURMIN			400u
#define EIGHTMIN		800u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CONV_TO_MINUTES_COMP(x)   ((x / ONEMIN) % 60)  // complement with hours
#define CONV_TO_MINUTES(x)   ((float)x / (float)ONEMIN)     // just minutes
#define GET_SECONDS(x)      (((float)CONV_TO_MINUTES(x) - (uint32_t)CONV_TO_MINUTES(x)) * 60) // extract seconds from minutes
#define CONV_TO_HOURS(x)    ((x / ONEHOUR) % 12)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t start_day;  		// start of day
uint32_t end_day;    		// end of day
uint8_t done = 0;
uint32_t idlehookCount = 0;
TELLER_t tellers[3];
char constBuffer[100];
volatile METRICS_t metrics;
osThreadId_t tellerHandle_1;
osThreadId_t tellerHandle_2;
osThreadId_t tellerHandle_3;
osThreadId_t tellerHandles[3];
const osThreadAttr_t teller_attributes_1 = {
  .name = "TELLERTASK1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
const osThreadAttr_t teller_attributes_2 = {
  .name = "TELLERTASK2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
const osThreadAttr_t teller_attributes_3 = {
  .name = "TELLERTASK3",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE END Variables */
/* Definitions for bank */
osThreadId_t bankHandle;
const osThreadAttr_t bank_attributes = {
  .name = "bank",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for customers */
osMessageQueueId_t customersHandle;
uint8_t customersBuffer[ 69 * sizeof( uint32_t ) ];
osStaticMessageQDef_t customersControlBlock;
const osMessageQueueAttr_t customers_attributes = {
  .name = "customers",
  .cb_mem = &customersControlBlock,
  .cb_size = sizeof(customersControlBlock),
  .mq_mem = &customersBuffer,
  .mq_size = sizeof(customersBuffer)
};
/* Definitions for metricMutex */
osMutexId_t metricMutexHandle;
const osMutexAttr_t metricMutex_attributes = {
  .name = "metricMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void teller_task(void *argument);
void print_metrics(void);
void printLine(uint8_t* buffer);
/* USER CODE END FunctionPrototypes */

void bank_op(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	idlehookCount++;
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */


  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of metricMutex */
  metricMutexHandle = osMutexNew(&metricMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of customers */
  customersHandle = osMessageQueueNew (69, sizeof(uint32_t), &customers_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bank */
  bankHandle = osThreadNew(bank_op, NULL, &bank_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  teller_task_init(3);  // create 3 tellers
	for(int ii=0; ii<NUM_TELLERS; ii++) {
	  TELLER_t *t = &tellers[ii];
	  memset(t, 0, sizeof(TELLER_t));
	  t->instance = ii;
	  t->onBreak = 0;	// scheduled break
	  t->breakTime = 50; // default wait time if no one is in line
	  t->breakOverride = 0; // button break
	  t->numBreaks = 0;
	  t->shortestBreak = 99999999;
	  t->longestBreak = 0;
	  t->averageBreak = 0;
	  t->status = 'I';
	//	  tellerHandle = osThreadNew(teller_task, &tellers[ii],&teller_attributes );
	}
	tellerHandle_1 = osThreadNew(teller_task, &tellers[0],&teller_attributes_1 );
	tellerHandle_2 = osThreadNew(teller_task, &tellers[1],&teller_attributes_2 );
	tellerHandle_3 = osThreadNew(teller_task, &tellers[2],&teller_attributes_3 );
	tellerHandles[0] = tellerHandle_1;
	tellerHandles[1] = tellerHandle_2;
	tellerHandles[2] = tellerHandle_3;
	start_day = xTaskGetTickCount();
	end_day = start_day + ONEDAY;


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_bank_op */
/**
  * @brief  Function implementing the bank thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_bank_op */
void bank_op(void *argument)
{
  /* USER CODE BEGIN bank_op */
	uint32_t randomNum = 0;

	uint32_t queueVal = xTaskGetTickCount();
	MFS_init();
	uint16_t numCustomers = 0;
	uint16_t maxCustomers = 0;
	sprintf(constBuffer, "\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\f");  // clears screen
	printLine((uint8_t*)constBuffer);
	sprintf(constBuffer, "                        Teller Status: B->Busy, X->Break, I->Idle\r\n");  // clears screen
	printLine((uint8_t*)constBuffer);
  /* Infinite loop */
  while(1)
  {
	  while(!done){
		  if ((xTaskGetTickCount() < end_day) && (xTaskGetTickCount() > (queueVal + randomNum))) {
			  // get time ticks
			  queueVal = xTaskGetTickCount();
			  // enqueue 1 customer with GetTick value
			  osMessageQueuePut(customersHandle, &queueVal, 0, 0);
			  // generate RNG sleep value
			  HAL_RNG_GenerateRandomNumber(&hrng, &randomNum);
			  randomNum = ((randomNum % (FOURMIN - ONEMIN)) + ONEMIN); // 1min to 4min
		//		  osDelay(randomNum);
		  }
		  // update number in queue
		  numCustomers = osMessageQueueGetCount(customersHandle);
		  uint8_t t1 = tellers[0].status;
		  uint8_t t2 = tellers[1].status;
		  uint8_t t3 = tellers[2].status;
		  uint32_t curr_time1 = xTaskGetTickCount();
		  sprintf(constBuffer, "Time: %02ld:%02ld, Queue: %d, T1[ %c ]: %d, T2[ %c ]:%d, T3[ %c ]:%d         \r",
				  (CONV_TO_HOURS((curr_time1 + (9*ONEHOUR)))),
				  CONV_TO_MINUTES_COMP(curr_time1),
				  numCustomers,
//				  'I',
				  (char)t1,
				  tellers[0].customers_served,
//				  'I',
				  (char)t2,
				  tellers[1].customers_served,
//				  'I',
				  (char)t3,
				  tellers[2].customers_served);
		  printLine((uint8_t*)constBuffer);

		  // update global max queue depth metric
		  if (numCustomers > maxCustomers) {
			  maxCustomers = numCustomers;
			  osMutexAcquire(metricMutexHandle, 0xFFFFFF);
			  metrics.max_queue_depth = numCustomers;
			  osMutexRelease(metricMutexHandle);
		  }
		  // update MFS
		  MFS_print_int(numCustomers);
		  MFS_7seg_refresh();
		//	  MFS_7seg_refresh();
		  osDelay(1); // arbitrary amount for LED refresh smoothness
		  if ((xTaskGetTickCount() > end_day) && (numCustomers == 0)){
			  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\nDAY IS OVER\r\n", 16, ~0);
			  //  osDelay(1000); // give time for last customers
			  print_metrics(); // print end of day metrics;
			  done = 1;
			  break;
		  }
	  }
	  osDelay(5000);
  }
//  osThreadTerminate(bankHandle);
  /* USER CODE END bank_op */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void teller_task(void *argument) {
    TELLER_t *t = (TELLER_t *)argument;
    int instance = t->instance+1;   // which teller am I?  1, 2, or 3?
    // global metric variables
    uint32_t randomNum;
    uint32_t retrieved; // QUEUE VALUE
    uint32_t customer_wait;
    uint32_t teller_wait;
    uint32_t last_served_time = 0;

    // misc variables
    uint32_t curr_time = 0;
    uint8_t karen = 0; // person still in line after workday overs

    // break variables
    uint32_t breakTimeInterval;  // 1 to 4 minutes
    uint32_t lastBreakTime = 0;
    uint32_t nextBreakGen;  // 30 to 60 minutes
    uint8_t wasPressed = 0;

    // teller metric variables
    uint32_t break_time;



    HAL_RNG_GenerateRandomNumber(&hrng, &breakTimeInterval);
    breakTimeInterval = ((breakTimeInterval % ((ONEMIN*4) - ONEMIN)) + ONEMIN);
    t->breakTime = breakTimeInterval;
    HAL_RNG_GenerateRandomNumber(&hrng, &nextBreakGen);
    nextBreakGen = ((nextBreakGen % ((ONEHOUR) - (ONEHOUR/2))) + (ONEHOUR/2));
    while (1) {
    while(xTaskGetTickCount() < end_day || karen) {
    	curr_time = xTaskGetTickCount(); // measure for later
    	if (MFS_button_pressed(instance) && (curr_time > 10)){
    		t->breakOverride = 1;
    		t->onBreak = 1;
    		t->status = 'X';
    		if (wasPressed == 0){
    		    			lastBreakTime = curr_time;
    		    		}
    		wasPressed = 1;
    	}
    	else {
    		t->breakOverride = 0;
    	}
    	MFS_set_led(instance, (t->onBreak || t->breakOverride));
    	if ((curr_time > (lastBreakTime + nextBreakGen)) && (!t->breakOverride)) {
    		t->onBreak = 1;
    		t->status = 'X';
    		if (wasPressed == 0){
    		    			lastBreakTime = curr_time;
    		    		}
    		HAL_RNG_GenerateRandomNumber(&hrng, &breakTimeInterval);
			breakTimeInterval = ((breakTimeInterval % ((ONEMIN*4) - ONEMIN)) + ONEMIN);
			t->breakTime = breakTimeInterval;
			HAL_RNG_GenerateRandomNumber(&hrng, &nextBreakGen);
			nextBreakGen = ((nextBreakGen % ((ONEHOUR) - (ONEHOUR/2))) + (ONEHOUR/2));
    	}
    	if (t->onBreak == 0 && (osMessageQueueGetCount(customersHandle) != 0)) {
    		// put tellers on break if buttons pressed
			// Dequeue from customer queue
    		t->status = 'B'; // busy
    		osMessageQueueGet(customersHandle, &retrieved, NULL, 0);
    		t->customers_served++;

			// Generate RNG
    		HAL_RNG_GenerateRandomNumber(&hrng, &randomNum);
    		randomNum = ((randomNum % (EIGHTMIN - THIRTYSEC)) + THIRTYSEC); // 30sec to 8min trans time



//			 print a newline every second
//			if(instance==1)
//				HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, ~0);



			customer_wait = curr_time - retrieved; // wait time for current customer
			teller_wait = curr_time - last_served_time; // wait for teller
			last_served_time = curr_time;
			// Update Metrics (volatile)
			osMutexAcquire(metricMutexHandle, portMAX_DELAY);
			metrics.customers_served++;
			// running average for customer wait
			metrics.avg_wait_time = (metrics.avg_wait_time * (metrics.customers_served - 1) + customer_wait) / metrics.customers_served;
			// running average for teller wait
			metrics.avg_teller_wait = (metrics.avg_teller_wait * (metrics.customers_served - 1) + teller_wait) / metrics.customers_served;

			// running average for transaction time
			metrics.avg_trans_time = (metrics.avg_trans_time * (metrics.customers_served - 1) + randomNum) / metrics.customers_served;

			// check max for all above and update if necessary
			if (customer_wait > metrics.max_wait_time) (metrics.max_wait_time = customer_wait);
			if (teller_wait > metrics.max_teller_wait) (metrics.max_teller_wait = teller_wait);
			if (randomNum > metrics.max_trans_time) (metrics.max_trans_time = randomNum);
			osMutexRelease(metricMutexHandle);
			// end of update for metrics


			(osMessageQueueGetCount(customersHandle) != 0) ? (karen = 1) : (karen = 0);
			osDelay(randomNum);
	//        vTaskDelay(portMAX_DELAY);
    	}

    	else {


			// check for button being released
			if (!t->breakOverride){
				if (wasPressed) {
					wasPressed = 0;
					t->onBreak = 0;
					t->status = 'I';
					t->numBreaks++;
					break_time = curr_time - lastBreakTime;
					t->averageBreak = (t->averageBreak * (t->numBreaks - 1) + break_time) / t->numBreaks;
					if (break_time > t->longestBreak) (t->longestBreak = break_time);
					if (break_time < t->shortestBreak) (t->shortestBreak = break_time);
				}
			}


			if ((curr_time > (lastBreakTime + t->breakTime)) && (t->onBreak) && (!t->breakOverride)){ // scheduled break is up
				t->onBreak = 0;
				t->status = 'I';
				// since break is over, record break time metrics
				break_time = curr_time - lastBreakTime;
				t->numBreaks++;
				t->averageBreak = (t->averageBreak * (t->numBreaks - 1) + break_time) / t->numBreaks;
				if (break_time > t->longestBreak) (t->longestBreak = break_time);
				if (break_time < t->shortestBreak) (t->shortestBreak = break_time);
			}

			osDelay(5); // either no customer in line or is on break, wait
    	}
    }
    osDelay(portMAX_DELAY);
    }
//    osThreadTerminate(&tellerHandles[t->instance]);
}

void print_metrics() {
	uint8_t messageBuffer[150];
	sprintf((char*)messageBuffer, "\r\n\r\n END OF DAY METRICS \r\n");
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Total Customers Served:                %ld \r\n", metrics.customers_served);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Total Customers Served by Teller 1:    %d \r\n", tellers[0].customers_served);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Total Customers Served by Teller 2:    %d \r\n", tellers[1].customers_served);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Total Customers Served by Teller 3:    %d \r\n", tellers[2].customers_served);
	printLine(messageBuffer);

	sprintf((char*)messageBuffer, "Average Customer Waiting Time:         %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.avg_wait_time), (uint32_t)GET_SECONDS(metrics.avg_wait_time));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Average Transaction Time:              %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.avg_trans_time), (uint32_t)GET_SECONDS(metrics.avg_trans_time));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Average Teller Wait Time:              %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.avg_teller_wait), (uint32_t)GET_SECONDS(metrics.avg_teller_wait));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Max Customer Wait Time:                %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.max_wait_time), (uint32_t)GET_SECONDS(metrics.max_wait_time));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Max Transaction Time:                  %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.max_trans_time), (uint32_t)GET_SECONDS(metrics.max_trans_time));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Max Teller Wait Time:                  %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(metrics.max_teller_wait), (uint32_t)GET_SECONDS(metrics.max_teller_wait));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Max Queue Depth:                       %ld \r\n", metrics.max_queue_depth);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Idle Hook Count:                       %ld \r\n", idlehookCount);
	printLine(messageBuffer);


	sprintf((char*)messageBuffer, "\r\n\r\n GRADUATE METRICS (Break Time Metrics) \r\n");
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Number of Breaks (Teller 1):           %d \r\n", tellers[0].numBreaks);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Average Break Time (Teller 1):         %ld minutes and %ld seconds  \r\n", (uint32_t)CONV_TO_MINUTES(tellers[0].averageBreak), (uint32_t)GET_SECONDS(metrics.max_teller_wait));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Longest Break Time (Teller 1):         %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[0].longestBreak), (uint32_t)GET_SECONDS(tellers[0].longestBreak));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Shortest Break Time (Teller 1):        %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[0].shortestBreak), (uint32_t)GET_SECONDS(tellers[0].shortestBreak));
	printLine(messageBuffer);

	sprintf((char*)messageBuffer, "Number of Breaks (Teller 2):           %d \r\n", tellers[1].numBreaks);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Average Break Time (Teller 2):         %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[1].averageBreak), (uint32_t)GET_SECONDS(tellers[1].averageBreak));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Longest Break Time (Teller 2):         %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[1].longestBreak), (uint32_t)GET_SECONDS(tellers[1].longestBreak));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Shortest Break Time (Teller 2):        %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[1].shortestBreak), (uint32_t)GET_SECONDS(tellers[1].shortestBreak));
	printLine(messageBuffer);

	sprintf((char*)messageBuffer, "Number of Breaks (Teller 3):           %d   \r\n", tellers[2].numBreaks);
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Average Break Time (Teller 3):         %ld minutes and %ld seconds \r\n", (uint32_t)CONV_TO_MINUTES(tellers[2].averageBreak), (uint32_t)GET_SECONDS(tellers[2].averageBreak));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Longest Break Time (Teller 3):         %ld minutes and %ld seconds   \r\n", (uint32_t)CONV_TO_MINUTES(tellers[2].longestBreak), (uint32_t)GET_SECONDS(tellers[2].longestBreak));
	printLine(messageBuffer);
	sprintf((char*)messageBuffer, "Shortest Break Time (Teller 3):        %ld minutes and %ld seconds  \r\n", (uint32_t)CONV_TO_MINUTES(tellers[2].shortestBreak), (uint32_t)GET_SECONDS(tellers[2].shortestBreak));
	printLine(messageBuffer);

}

void printLine(uint8_t* buffer) {
	HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 100);
}

/* USER CODE END Application */

