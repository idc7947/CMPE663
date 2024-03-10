/*
 * button.c
 *
 *  Created on: Jul 16, 2021
 *      Author: rickweil
 */

#include "stm32l476xx.h"

// On the NUCLEO board, the blue User button B1 is connected to PC13 of the MCU (Port C, Bit 13).
// When the button is pressed, bit 13 will be low.  It will be high otherwise
#define BUTTON_PIN 13
#define BUTTON_PORT GPIOC
void button_init(void)
{
	// Configure BUTTON_PIN as an input
    BUTTON_PORT->MODER &= ~(0x3<<(BUTTON_PIN*2)) ;        // clear the two MODE bits for this pin to make an input

    // ...with a pull-up resistor
    BUTTON_PORT->PUPDR &= ~(0x3<<(BUTTON_PIN*2)) ;		// first, clear the two PUPDR bits for this pin
    BUTTON_PORT->PUPDR |= 0x2 << (BUTTON_PIN*2);
}

_Bool button_isPressed(void )
{
	uint32_t bit = BUTTON_PORT->IDR & (1<<BUTTON_PIN);
	return (bit != 0);
}
