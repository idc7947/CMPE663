/*
 * commands.c
 *
 *  Created on: Feb 26, 2023
 *      Author: iandc
 */
#include "commands.h"
#include "stm32l4xx_hal.h"

// default recipe
uint8_t recipe1[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		WAIT | 5,
		MOV  | 1,
		WAIT | 4,
		MOV  | 2,
		WAIT | 3,
		MOV  | 3,
		WAIT | 2,
		MOV  | 4,
		WAIT | 1,
		MOV  | 5,
		LOOP | 4,	// Test the default loop behavior.
		MOV  | 1,
		WAIT | 5,
		MOV  | 5,	  // Is there full travel?
		LOOP_END,
		MOV  | 0,
		WAIT | 31,	// Measure the timing precision of the
		WAIT | 31,	// 9.6 second delay with an
		WAIT | 31,	// external timer.
		MOV  | 4,
		RECIPE_END
};

// recipe for delay testing
uint8_t recipe2[RECIPE_SIZE] = 	{
		WAIT | 31,
		MOV  | 0,	// Check all positions and wait times
		MOV  | 5,
		WAIT | 31,	// Measure the timing precision of the
		WAIT | 31,	// 9.6 second delay with an
		WAIT | 31,	// external timer.
		MOV  | 4,
		RECIPE_END
};

// recipe with print statements
uint8_t recipe3[RECIPE_SIZE] = 	{
		MOV  | 5,
		MOV	 | 0,
		MOV  | 3,
		OPPOSITE,
		WAIT | 5,
		MOV  | 1,
		WAIT | 4,
		MOV  | 2,
		OPPOSITE,
		WAIT | 3,
		MOV  | 3,
		WAIT | 2,
		MOV  | 4,
		WAIT | 1,
		MOV  | 5,
		LOOP | 4,	// Test the default loop behavior.
		MOV  | 1,
		WAIT | 5,
		MOV  | 5,	  // Is there full travel?
		LOOP_END,
		MOV  | 0,
		WAIT | 31,	// Measure the timing precision of the
		WAIT | 31,	// 9.6 second delay with an
		WAIT | 31,	// external timer.
		MOV  | 4,
		RECIPE_END
};

uint8_t recipe4[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		WAIT | 5,
		MOV  | 1,
		WAIT | 4,
		MOV  | 2,
		WAIT | 3,
		MOV  | 3,
		LOOP	,
		WAIT | 2,
		MOV  | 4,
		LOOP	,  // Loop error
		WAIT | 1,
		MOV  | 5,
		RECIPE_END
};

uint8_t recipe5[RECIPE_SIZE] = 	{
		MOV  | 8,  // out of range error
		RECIPE_END
};

uint8_t recipe6[RECIPE_SIZE] = 	{
		WAIT | 5,
		MOV  | 1,
		WAIT | 4,
		MOV  | 2,
		WAIT | 3,
		MOV  | 3,
		WAIT | 2,
		MOV  | 4,
		WAIT | 5,
		MOV  | 1,
		WAIT | 4,
		MOV  | 2,
		WAIT | 3,
		MOV  | 3,
		WAIT | 2,
		MOV  | 0,	// Check all positions and wait times
		RECIPE_END
};

uint8_t recipe7[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		RECIPE_END
};

uint8_t recipe8[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		RECIPE_END
};

uint8_t recipe9[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		RECIPE_END
};

uint8_t recipe10[RECIPE_SIZE] = 	{
		MOV  | 0,	// Check all positions and wait times
		RECIPE_END
};

uint8_t* recipes[] = {
		recipe1,
		recipe2,
		recipe3,
		recipe4,
		recipe5,
		recipe6,
		recipe7,
		recipe8,
		recipe9,
		recipe10,
		NULL
};


void execute_input(Servo* servo, uint8_t command){
	switch(command){
		case 'P':		// clear paused flag, allow for override
			servo->is_paused = 1;
			servo->is_running = 0;
			MFS_set_led(servo->servo_num, 0);
			break;
		case 'C':		// set paused flag, don't allow override
			if (servo->is_paused && !servo->error) {
				servo->is_paused = 0;
				servo->is_running = 1;
				MFS_set_led(servo->servo_num, 1);
			}
			break;
		case 'R':		// assert override command of servo_1, determine if valid
			if (servo->is_paused) moveRight(servo);
			servo->last_instr_time = HAL_GetTick();
			break;
		case 'L':		// assert override command of servo_1, determine if valid
			if (servo->is_paused) moveLeft(servo);
			servo->last_instr_time = HAL_GetTick();
			break;
		case 'N':		// Do none :)
			break;
		case 'B':		// Set index to 0 to restart recipe
			if (servo->is_paused) {
				servo->error = 0;
				MFS_set_led(servo->servo_num, 1);
				MFS_set_led(3, 0);
				servo->recipe_index = 0;
				servo->is_running = 1;
			}
			break;
		default:		// set recipe index
			if (servo->is_paused) {
				if (isdigit(command)) {
					servo->recipe_num = atoi((char*)&command);
					servo->recipe_index = 0;
				}
			}
			break;
	}
}


uint8_t execute_recipe_step(Servo* servo){
	uint8_t curr_recipe[50];
	memcpy(curr_recipe, select_recipe(servo->recipe_num), 50);
	uint8_t curr_command = curr_recipe[servo->recipe_index];
	uint8_t op = (curr_command & OP_MASK);
	uint8_t param = (curr_command & PARAM_MASK);
	MFS_set_led(servo->servo_num, 1);
	if (servo->servo_num == 1) {
		MFS_set_led(4, 0);
		MFS_set_led(3, 0); // clears error LEDs
		servo->error = 0;
	}
	switch(op) {
		case OPPOSITE: // prints status
			if (servo->servo_pos == 0) movePos(servo, 5);
			if (servo->servo_pos == 1) movePos(servo, 4);
			if (servo->servo_pos == 2) movePos(servo, 3);
			if (servo->servo_pos == 3) movePos(servo, 2);
			if (servo->servo_pos == 4) movePos(servo, 1);
			if (servo->servo_pos == 5) movePos(servo, 0);
			servo->recipe_index++;
			return 0;  // print command sent back to where uart is accessible
			break;
		case MOV:
			if (param < 6 && param >= 0) {
				movePos(servo, param);
				servo->recipe_index++;
			}
			else {
				if (servo->servo_num == 1){
					MFS_set_led(3, 1);  // if out of range set error
				}
				MFS_set_led(servo->servo_num, 0);
				servo->error = 1;
				servo->is_running = 0;
				servo->is_paused = 1;
				servo->recipe_index = 0;
			}
			break;
		case WAIT:
			servo->delay+=(param*100);
			servo->recipe_index++;
			break;
		case LOOP:
			if (servo->in_loop) {
				servo->loop_index = 0; // encountered nested loop, turn on LED 4 for servo_1
				if (servo->servo_num == 1) MFS_set_led(4, 1);
				MFS_set_led(servo->servo_num, 0);
				servo->is_paused = 1;
				servo->is_running = 0;
				servo->error = 1;
				servo->recipe_index = 0;
				return 0;
			}
			servo->in_loop = 1;
			servo->loop_index = servo->recipe_index + 1;
			servo->loop_iterations = param;
			servo->recipe_index++;
			break;
		case LOOP_END:
			if (servo->loop_iterations == 0) {
				servo->in_loop = 0;
				servo->recipe_index++;
			}
			else {
				servo->loop_iterations--;
				servo->recipe_index = servo->loop_index;
			}
			break;
		case RECIPE_END:
			servo->is_running = 0;
			servo->is_paused = 1;
			servo->error = 0;
			MFS_set_led(servo->servo_num, 0);
			break;
		default:
			if (servo->servo_num == 1) MFS_set_led(3, 1);
			servo->is_running = 0;
			servo->is_paused = 1;
			servo->recipe_index = 0;
			break;
	}
	return 0; // no print

}

uint8_t* select_recipe(uint8_t index){
	switch(index) {
		case 0:
			return recipe1;
			break;
		case 1:
			return recipe2;
			break;
		case 2:
			return recipe3;
			break;
		case 3:
			return recipe4;
			break;
		case 4:
			return recipe5;
			break;
		case 5:
			return recipe6;
			break;
		case 6:
			return recipe7;
			break;
		case 7:
			return recipe8;
			break;
		case 8:
			return recipe9;
			break;
		case 9:
			return recipe1;
			break;
		default:
			return recipe1;
			break;
	}

}
