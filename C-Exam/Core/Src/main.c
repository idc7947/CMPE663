#include "stm32l476xx.h"
#include "clock.h"
#include "MFS.h" // functionality for MFS shield (LEDs, buttons, do not change)

// These are function prototypes for this C exam -- do not change
void function_one( void ) ;
void function_two( void ) ;
void function_three( void ) ;

// Checks for button presses and acts as described below
// Calls function_one when button 1 is pressed.
// Calls function_two when button 2 is pressed.
//
// DO NOT CHANGE this function.
void check_buttons()
{
    // Call function 1 when button 1 is pushed
	if(MFS_button_pressed(1)){
		function_one();
	}

    // Call function 2 when button 2 is pushed
	else if( MFS_button_pressed(2) ){
		function_two();
	}
}

// Initializes TIM2 to count at 1MHz
//
// DO NOT CHANGE this function.
static void TIM2_init( void )
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;	// enable clock for this timer in the clock control
	TIM2->PSC = 15999;						// load the prescaler value -- divide 16 MHz clock down to 1 kHz
	TIM2->EGR |= TIM_EGR_UG ;				// force an update event to make the prescaler load take effect
	TIM2->CR1 |= TIM_CR1_CEN ;				// now start the timer
}

// Returns the current count value from timer TIM2.
//
// DO NOT CHANGE this function.
static uint32_t TIM2_get_count()
{
	return TIM2->CNT ;
}

// Initializes the system and then does an infinite loop
// that checks the MFS buttons and calls function_three()
//
// DO NOT CHANGE this function.
int main(void){

	clock_init();			// initialize the system clock
	TIM2_init();			// start TIM2 running at 1kHz
	MFS_init();				// initialize the MultiFunction Shield

	while (1)
	{
		check_buttons();	// see if any buttons pressed
		function_three();	// YOU write function_three (and _one and _two)
	}
}

// DO NOT CHANGE ANYTHING ABOVE THIS LINE

///////////////////////////////////////////////////////////////////////////////
// Implement the three functions described below. You are allowed to         //
// add data declarations outside the functions if you find that helpful.     //
// You may call any of the MFS or above functions.                           //
//                                                                           //
// Enter your code below this comment block. Do not change ANYTHING          //
// above this comment block or in any other file in this project.            //
//                                                                           //
// Your functions MUST NOT BLOCK the execution of the while loop in main.    //
///////////////////////////////////////////////////////////////////////////////

// This function is called when button 1 is pressed.
//
// The purpose of this function is to disable flashing of the LEDs in function_three.
// An appropriate name for this function would be "stop".
// Tip -- Do NOT change the LEDs in this function.
//        Your code in this function should be very simple!
uint32_t currLED = 1;
uint32_t enable = 1;
uint32_t disable = 0;
uint32_t prevCount = 0;
void function_one()
{
	enable = 0;
}


// This function is called when the button 2 is pressed.
//
// The purpose of this function is to enable flashing of the LED in function_three.
// An appropriate name for this function would be "start".
// Tip -- Do NOT change the LEDs in this function.
//        Your code in this function should be very simple!

void function_two()
{
	enable = 1;

}

// This function is called from the infinite loop in main after the buttons are checked.
//
// function_three() must implement a simple 4 LED walking light sequence where
//    1. for 256 msec, only LED1 is on, the others are off, then
//    2. for 256 msec, only LED2 is on, the others are off, then
//    3. for 256 msec, only LED3 is on, the others are off, then
//    4. for 256 msec, only LED4 is on, the others are off, then
//    4. keep repeating this cycle forever from step 1 thru 4.
// In other words, light LED 1,2,3,4,1,2,3,4,1...
//
// NOTE -- your code will be inspected to confirm this exact timing and lighting.
// Tip -- used the functions provided above and in mfs.c!
void function_three()
{

	MFS_set_led(currLED, enable);
	if ((TIM2_get_count() - prevCount) > 0x100) {
		MFS_set_led(currLED, disable);
		currLED++;
		prevCount = TIM2_get_count();
	}
	if (currLED > 4) currLED = 1;

}
