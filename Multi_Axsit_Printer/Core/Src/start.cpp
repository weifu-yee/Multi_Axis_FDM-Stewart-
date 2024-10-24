/*
 * start.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: sunny
 */

#include "start.h"
#include "gpio.h"
#include <stdio.h>

class START Start;
bool started;

START::START(){}
START::~START(){}

void START::init(){
	started = false;
}

void START::getStarted(){
	started = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == START_GPIO_PIN){
        Start.getStarted();
    }
}


