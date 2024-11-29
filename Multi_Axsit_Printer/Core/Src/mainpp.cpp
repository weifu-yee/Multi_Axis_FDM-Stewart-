/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

/*INCLUDES*/
#include "mainpp.h"
#include <stdio.h>
#include "arduino.h"
#include "main.h"
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"
#include "timing.h"
#include "start.h"
#include "determine_KP_mode.h"

int line_of_Gcode = 0;
bool reached = true;
bool determine_KP_mode = false;

void Timer_INIT(void) {
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim23, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim24, TIM_CHANNEL_ALL);
}

extern int _c;
extern double mod_Kp[7];
extern bool stop;

void main_function(void){
	Start.init();
	HAL_GPIO_WritePin(MM_Enable_GPIO_PORT_1, MM_Enable_GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MM_Enable_GPIO_PORT_2, MM_Enable_GPIO_PIN_2, GPIO_PIN_RESET);
	Timer_INIT();
	initialize_platform();

	reset_pushers_to_home();
	Arduino.init();

	while(!started);
	HAL_Delay(50);
	started = false;


	determine_KP_mode = false; //switch true or false
	if (determine_KP_mode) determine_KP_loop(1); //1 for rectangular or 2 for elevator

	while(1){
		printf("Hello %d \n", line_of_Gcode);
		line_of_Gcode++;
		if (!stop) {
			Arduino.readGcode();
		}

		prev_diffNorm = 0;
		increasing_count = 0;
		prev_SPerror = 0;
		SPerror_increasing_count = 0;

		reached = false;
		while(!reached){}; //waiting the process in timing.cpp

//		if() { //when M2 end of file been delivered.
//
//		}

		//this while is for debug, lock the process between each line of Gcode.
//		while(_c != 0){}
//		_c ++;

		while(stop){}; //waiting press the ArduinoRest button, and press start
		HAL_Delay(50);
	}
}




