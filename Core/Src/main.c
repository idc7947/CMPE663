#include "stm32l476xx.h"
#include "timer.h"
#include "clock.h"
#include "GPIO.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <uart.h>
#define MAX_PRINT_BUFFER 69
char message[50] = "Enter desired period, or press enter. \r\n";    // message to print to the user
char messageString[150];
char lineBuffer[150];
char buffer[20];	     // holds the user response
uint32_t buckets[101];
uint32_t non_zero_times[1000];
uint32_t num_non_zero = 0;
uint32_t non_zero_size = 0;
uint32_t lower_limit = 1000 - 50;	// the default lower limit in the problem statement
uint32_t lastCapture = 0;

//////////////////////////////////////////////////////////////
// Function declarations
////////////////

// runs the power on self-test. Returns true if the test passes, false otherwise
_Bool power_on_self_test( void );

// initializes all variables prior to a measurement.
int init_measurement( uint32_t limit );

// measures timing of 1000 rising edges.
int make_measurements( uint32_t limit );

// print the non-zero bucket times and counts
int print_measurements( uint32_t limit );

// Captures 1 line of text from the console. Returns nul terminated string when \n is entered
void get_line ( char *bufferParam, int max_length );

// Parses a line of user input into a new lower limit (unchanged if no response or invalid response)
void get_limit ( char *bufferParam);
void printFunct(char* printBuffer);
uint32_t TIM2_EdgeToEdge(void);

// initializes the timer (Similar to USART2_Init().  Place in timer.c)
//void TIM_Init(void);
//void GPIO_Init(void);

//////////////////////////////////////////////////////////////
// Embedded code usually consists of 2 components
//  - The init section is run once at startup and initializes all low level drivers and modules
//  - The main loop runs forever and calls the application tasks repeatedly.
////////////////
int main(void) {
    clock_init();
    USART2_Init(115200);
    GPIO_Init();

    printFunct("\r\n\r\n\r\n");
    printFunct(message);


    TIM_Init();
//    TIM2->SR &= ~TIM_SR_CC1IF; // clear status register
//    printFunct("TEST");
    uint8_t pass = 0;
    while( pass == 0) {
    	pass = power_on_self_test();
    	if (pass == 0) {
    		printFunct("POST Failed, hook up input to AO and press any key to try again...\r\n");
    		USART_Read(USART2);
    	}
    }


    //////////
    // Main loop runs forever
    //////////
    while(1)
    {
    	printFunct("Enter desired lower limit from 100 to 10000, or press enter \r\n");
    	// 1. Print “Enter expected period or <CR> if no change”. Wait for user response
    	get_line((char*)buffer, sizeof(buffer));


        // 3. measure 1000 pulses
		init_measurement( lower_limit );
		make_measurements( lower_limit );

        // 4. print time/count for non-zero counts
		print_measurements( lower_limit );
    }
}


//////////////////////////////////////////////////////////////
// Function implementation stubs
////////////////


// runs the power on self-test. Returns true if the test passes, false otherwise
_Bool power_on_self_test( void ) {
	uint32_t start_time = TIM2->CNT;
	  while ((TIM2->CNT - start_time) < 100) {
//		  printFunct("Checking POST...");
	    // Check if a rising edge has been detected
		  if (TIM2->SR & TIM_SR_CC1IF) {
	    	printFunct("POST Succeeded!\r\n");
	      return 1; // A pulse was detected, return success
	    }
	  }
	  return 0; // No pulse was detected, return failure
}

// initializes all variables prior to a measurement.
int init_measurement( uint32_t limit ) {
	for (int i = 0; i < 101; i++) {
		buckets[i] = 0;
	}
	lastCapture = 0;
	return 0;
}

uint32_t TIM2_EdgeToEdge(void) {
	while(!(TIM2->SR & TIM_SR_CC1IF)) {
		; // block until flag set
	}
	uint32_t currentCapture = TIM2->CCR1;
	uint32_t diff = currentCapture - lastCapture;
	lastCapture = currentCapture;
	// Clear the CC1F flag
//	TIM2->SR &= ~TIM_SR_CC1IF;
	return diff;
}

int make_measurements( uint32_t limit ) {

	  for (int i = 0; i <= 1000; i++) { // Take 1000 measurements
		  uint32_t measurement = TIM2_EdgeToEdge();
		  if (measurement % limit < 101) {    // 100 buckets
			  buckets[measurement % limit]++;
		  }
	  }
	  printFunct("Measurements finished. If none shown, adjust period\r\n");
}

// print the non-zero bucket times and counts
int print_measurements( uint32_t limit ) {
	for (int i = 0; i < 101; i++) {
		if (buckets[i] != 0) {
			int n = sprintf(lineBuffer, "%lu %lu \r\n", i + limit, buckets[i]);
//			USART_Write(USART2, (uint8_t*) lineBuffer, n);
			printFunct(lineBuffer);
		}
	}
}

void printFunct(char* printBuffer) {
	USART_Write(USART2, (uint8_t*)printBuffer, strlen(printBuffer)); // simple print solution
}

// Captures 1 line of text from the console. Returns nul terminated string when \n is entered
void get_line ( char *bufferParam, int max_length ) {
	    uint32_t length = 0;
	    while (length < max_length) {
	        char c = USART_Read(USART2);
	        char c_cast[2];
	        c_cast[0] = c;
	        c_cast[1] = '\0';    // sending single character to buffer
	        printFunct((char*) c_cast);
	        bufferParam[length] = c;
	        length++;

	        if (c == '\r')
	            break;
	    }
	    get_limit(bufferParam);
	    return;
}

// Parses a line of user input into a new lower limit (unchanged if no response or invalid response)
void get_limit ( char* bufferParam) {
	uint32_t proposed_limit = atoi(bufferParam);   // checks valid input
	if (proposed_limit >= 100 && proposed_limit <= 10000) {
		int n = sprintf(messageString, "Lower limit changed from %lu to %lu \r \n", lower_limit, proposed_limit);
		USART_Write(USART2, (uint8_t*) messageString, n);
		lower_limit = proposed_limit;

	}
	return;
}

