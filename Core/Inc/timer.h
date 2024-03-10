#ifndef __TIMER_H
#define __TIMER_H

#include "stm32l476xx.h"

// initializes the timer (Similar to USART2_Init().  Place in timer.c)
void TIM_Init(void);
uint32_t getPreScaler(void);
void setPreScaler(uint32_t preVal);

#endif /* __TIMER_H */
