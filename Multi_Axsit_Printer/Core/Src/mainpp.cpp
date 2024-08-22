/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"

void main_function(void){
	int count = 0;
	while(1){
		count++;
		printf("Hello %d \n", count);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(1000);
	}
}



