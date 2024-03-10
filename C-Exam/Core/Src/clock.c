/*
 * clock.c
 *
 *  Created on: Jul 16, 2021
 *      Author: rickweil
 */


#include "stm32l476xx.h"

void clock_init(void) {
    RCC->CR |= ((uint32_t)RCC_CR_HSION);

    // wait until HSI is ready
    while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}

    // Select HSI as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;     // 01: HSI16 oscillator used as system clock

    // Wait till HSI is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 ) {;}

    // Enable the clock to GPIO Ports A, and B
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // enable clock some MFS buttons / LEDs
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;    // enable clock some MFS buttons / LEDs
}
