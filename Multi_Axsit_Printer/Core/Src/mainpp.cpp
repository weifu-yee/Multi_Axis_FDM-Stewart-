/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"

int cnt_5 = 0;
int t_sec = 0;
int count = 0;

bool reached = false;
Vector3D p[6], b[6];
SPPose current = create_default_stewart_platform();
SPPose target = create_default_stewart_platform();
ActuatorPID pusher[6];
double current_length[6], target_length[6];

void main_function(void){
	initialize_platform(p, b);
	while(1){
		count++;
		printf("Hello %d \n", count);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(500);


		//readGCode();
		//update_parameters(&target,)
		calculate_leg(&current, p, b, current_length);
		calculate_leg(&target, p, b, target_length);
		while(!reached);  //move_platform_to_target_pose(&current, &target)
		current = target;
		reached = false;
	}
}



