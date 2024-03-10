#include "timer.h"
#include "stm32l4xx_hal.h"
#define TIM2_EN 	0
static uint32_t prescaleVal = 15;

// initializes the timer (Similar to USART2_Init().  Place in timer.c)
void TIM_Init(void) {


	// Enable the clock for the timer peripheral
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Initialize the timer structure
	TIM2->PSC = prescaleVal; 				// Set the prescaler
	TIM2->EGR |= TIM_EGR_UG;				// Init counter from 0


	// Initialize the input capture structure
	TIM2->CCMR1 &= (~TIM_CCMR1_CC1S_0); // Input on channel 1
	TIM2->CCMR1 |= ( 1 << TIM_CCMR1_CC1S_Pos); // Input on channel 1
	TIM2->CCER &= ~TIM_CCER_CC1P;
	// TIM2 peripheral is configured to capture rising edge of the input signal
	TIM2->CCER |= TIM_CCER_CC1E; // Enable input capture

	// Enable the input capture interrupt
	//TIM2->DIER |= TIM_DIER_CC1IE;

	// Enable the timer
	TIM2->CR1 |= TIM_CR1_CEN;


	return;
}

uint32_t getPreScaler(void) {
	return prescaleVal;
}

void setPreScaler(uint32_t preVal){
	prescaleVal = preVal;
}
