#include "GPIO.h"
#include "stm32l476xx.h"


// initializes function gen GPIO pin
void GPIO_Init(void) {

	 // Enable clock for GPIOA peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure GPIOA0 in alternate function mode
	GPIOA->MODER &= ~GPIO_MODER_MODER0;
	GPIOA->MODER |= GPIO_MODER_MODER0_1;

	// Select the desired alternate function (AF) for the pin
	// This depends on the specific function generator and the pin's datasheet
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL0;
	GPIOA->AFR[0] |= (uint32_t)0x01; // Set alternate function to TIM2


	return;
}
