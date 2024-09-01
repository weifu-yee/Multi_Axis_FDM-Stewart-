/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

/*INCLUDES*/
#include "mainpp.h"
#include "arduino.h"
#include <stdio.h>

/*EXTERN VARIABLES*/

/*GLOBAL VARIABLES*/

/*FUNCTIONS*/
void main_function(void) {
	int count = 0;
	char send[] = "data321";
	Arduino.init();
	while (1) {
		printf("Hello %d \n", count);
		Arduino.sendData(send);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		count++;
		HAL_Delay(500);
	}
}

