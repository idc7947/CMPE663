/*
 * servo.c
 *
 *  Created on: Feb 19, 2023
 *      Author: iandc
 */

#include "servo.h"


Servo initServo(uint8_t num) {
	Servo servo;
	servo.servo_num = num;
	servo.servo_pos = 0;
	servo.last_instr_time = 0;  // ticks at last command execution
	servo.delay = 0;
	servo.is_running = 0;    // flag to keep track if it's mid-recipe execution
	servo.recipe_index = 0;  // Step of current recipe
	servo.recipe_num = 0;  // Current recipe number
	servo.return_status = 0;   // controls END_RECIPE and LOOP operations
	servo.in_loop = 0;    // determines if recipe has entered a loop
	servo.loop_index = 0;    // recipe index of loop variable
	servo.loop_iterations = 0;		// how many iterations of loop are left
	servo.is_paused = 1	;		// if cleared, allow for recipe execution
	servo.error = 0; // cleared if no error
	movePos(&servo, 0);
	return servo;

}

void moveRight(Servo *servo) {
	if (servo->servo_pos != 0) {
		movePos(servo, servo->servo_pos - 1);
		servo->delay+=200;
	}
}

void moveLeft(Servo *servo){
	if (servo->servo_pos != 5) {
		movePos(servo, servo->servo_pos + 1);
		servo->delay+=200;
	}
}


void movePos(Servo *servo, uint8_t pos){
	servo->delay+=(200*(abs(servo->servo_pos-pos)));
	servo->servo_pos = pos;
	switch (servo->servo_num){
	case 1:
		TIM3->CCR1 = calcPWM(pos);
		break;
	case 2:
		TIM3->CCR2 = calcPWM(pos);
		break;
	default:
		break;
	}
}

uint32_t calcPWM(uint8_t pos){
	return POS_BASE + (pos * POS_INCREMENT);
}

void printStatus(Servo *servo){

}
