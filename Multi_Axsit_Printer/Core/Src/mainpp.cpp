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

int count = 0;
bool reached = true;

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



extern int _c; //what is this?

void main_function(void){
	Start.init();
	initialize_platform();
	reset_pushers_to_home();
	init_lengths_array(current_lengths);
	init_lengths_array(next_lengths);
	Arduino.init();
	while(1){
		printf("Hello %d \n", count);
//		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		count++;
		Arduino.readGcode();

//		reached = false;
//		while(!reached){}; //waiting the process in timing.cpp
//		HAL_Delay(500);
	}
}




