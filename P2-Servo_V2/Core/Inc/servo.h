/*
 * servo.h
 *
 *  Created on: Feb 15, 2023
 *      Author: iandc
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>

#define POS_BASE		500
#define POS_INCREMENT	300

typedef struct Servo{
  uint8_t servo_num;
  uint8_t servo_pos;

  // Timer flags
  uint32_t last_instr_time;  // ticks at last command execution
  uint32_t delay;

  // Execution flags
  uint8_t is_running;    // flag to keep track if it's mid-recipe execution
  uint8_t recipe_index;  // Step of current recipe


  uint8_t recipe_num;  // Current recipe number

  uint8_t return_status;   // controls END_RECIPE and LOOP operations

  uint8_t in_loop;    // determines if recipe has entered a loop

  uint8_t loop_index;    // recipe index of loop variable

  int8_t  loop_iterations;		// how many iterations of loop are left

  uint8_t is_paused	;		// if cleared, allow for recipe execution

  uint8_t error;  // cleared when no error



} Servo;

Servo initServo(uint8_t num);

void moveRight(Servo* servo);

void moveLeft(Servo* servo);

void movePos(Servo* servo, uint8_t pos);

uint32_t calcPWM(uint8_t pos);
void printStatus(Servo *servo);



#endif /* INC_SERVO_H_ */
