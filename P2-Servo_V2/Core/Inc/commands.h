/*
 * commands.h
 *
 *  Created on: Feb 26, 2023
 *      Author: iandc
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_
#include "servo.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include "MFS.h"
#include "stm32l4xx_hal.h"

// shifts and masks for 8 bit commands
#define		OP_SHIFT	0x05
#define		OP_MASK		0xE0
#define		PARAM_MASK	0x1F

#define RECIPE_SIZE 50

// command ops
#define		MOV			(1 << OP_SHIFT)
#define		WAIT		(2 << OP_SHIFT)
#define		OPPOSITE	(3 << OP_SHIFT)
#define		LOOP		(4 << OP_SHIFT)
#define		LOOP_END	(5 << OP_SHIFT)
#define		RECIPE_END	(0 << OP_SHIFT)
#define		ILLEGAL_1	(6 << OP_SHIFT)
#define		ILLEGAL_2	(7 << OP_SHIFT)

// error LEDs
#define		RECIPE_ERROR		3;
#define		NESTED_LOOP			4;



void execute_input(Servo* servo, uint8_t command);

uint8_t execute_recipe_step(Servo* servo);


uint8_t* select_recipe(uint8_t index);

#endif /* INC_COMMANDS_H_ */
