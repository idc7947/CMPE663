//*************************************  NUCLEO-L476RG Development board 	 ***************************************************************************
// STM32 NUCLEO-64 development board with STM32L476RG MCU
// https://www.st.com/en/evaluation-tools/nucleo-l476rg.html
// See Table 23 in the NUCLEO Users Manual
//
// STM32L4:  STM32L476RG MCU = ARM Cortex-M4 + FPU + DSP,
//           LQFP64, 1 MB of Flash, 128 KB of SRAM
//           Instruction cache = 32 lines of 4x64 bits (1KB)
//           Data cache = 8 lines of 4x64 bits (256 B)
//
//****************************************************************************************************************

// System include files
#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"

// project include files
#include "clock.h"
#include "button.h"
#include "led.h"

// Your main function should be a few function calls to
// initialization code and then an endless loop that continuously runs the application code.
int main(void)
{
    clock_init();
    button_init();
    led_init();

    // begin main loop
    while(1)
	{
    	if(button_isPressed())
    		led_set( 1 );
    	else
    		led_set( 0 );
    }
}
