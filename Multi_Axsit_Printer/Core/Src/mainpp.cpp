/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"

int count = 0;
bool reached = false;
StewartPlatform current = create_default_stewart_platform();
StewartPlatform target = create_default_stewart_platform();

void main_function(void){
	initialize_platform(&current, p, b);
	while(1){
		count++;
		printf("Hello %d \n", count);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(500);

		//readGCode();
		//update_parameters(&target,)
		while(!reached);  //move_platform_to_target_pose(&current, &target)
		reached = false;
	}
}



