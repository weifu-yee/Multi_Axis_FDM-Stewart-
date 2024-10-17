/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

/*INCLUDES*/
#include "mainpp.h"
#include "arduino.h"
#include "main.h"
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"
#include "timing.h"

int count = 0;
bool reached = true;
double X, Y, Z, E, F, PHI, THETA, PSI;

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

void angularNormalizer(double *ang) {
	*ang = (double) fmod(*ang + M_PI, 2*M_PI) - M_PI;
}

int nomalizeAng = 0;

void readGCode(void){
	switch(count) {
		case 1:
			X = 0.0;
			Y = 0.0;
			Z = 250;
			PHI = 0.0;
			THETA = 0.0;
			PSI = 0.0;
			F = 100.0;
			break;
		case 2:
			X = 1.5;
			Y = 2.5;
			Z = 250;
			PHI = 10.0;
			THETA = 5.0;
			PSI = 2.0;
			F = 150.0;
			break;
		case 3:
			X = 20.0;
			Y = 30.0;
			Z = 250;
			PHI = 20.0;
			THETA = 10.0;
			PSI = 5.0;
			F = 200.0;
			break;
		case 4:
			X = 25.0;
			Y = 35.0;
			Z = 250;
			PHI = 30.0;
			THETA = 15.0;
			PSI = 10.0;
			F = 250.0;
			break;
		case 5:
			X = 30.0;
			Y = 40.0;
			Z = 250;
			PHI = 40.0;
			THETA = 20.0;
			PSI = 15.0;
			F = 300.0;
			break;
		case 6:
			X = 1.0;
			Y = 2.0;
			Z = 250;
			PHI = 10.0;
			THETA = 5.0;
			PSI = 0.0;
			F = 100.0;
			break;
		default:
			break;
	}
	angularNormalizer(&PHI);
	angularNormalizer(&THETA);
	angularNormalizer(&PSI);
	nomalizeAng++;
}
void update_parameters(void) {
	target.x = X;
	target.y = Y;
	target.z = Z;
	target.phi = PHI;
	target.theta = THETA;
	target.psi = PSI;
	double dx = X - current.x;
	double dy = Y - current.y;
	double dz = Z - current.z;
	double total_distance = sqrt(dx*dx + dy*dy + dz*dz);
	double time = total_distance / F;
	Velo.x = dx / time;
	Velo.y = dy / time;
	Velo.z = dz / time;

	double dphi = PHI - current.phi;
	double dtheta = THETA - current.theta;
	double dpsi = PSI - current.psi;
	// Normalize angular differences to [-π, π]
	dphi = fmod(dphi + M_PI, 2*M_PI) - M_PI;
	dtheta = fmod(dtheta + M_PI, 2*M_PI) - M_PI;
	dpsi = fmod(dpsi + M_PI, 2*M_PI) - M_PI;
	Velo.phi = dphi / time;
	Velo.theta = dtheta / time;
	Velo.psi = dpsi / time;
}

extern int _c;
extern double prev_diffNorm;
extern int increasing_count;
extern double prev_SPerror;
extern int SPerror_increasing_count;

void main_function(void){
	HAL_GPIO_WritePin(MM_Enable_GPIO_PORT_1, MM_Enable_GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MM_Enable_GPIO_PORT_2, MM_Enable_GPIO_PIN_2, GPIO_PIN_RESET);
//	_c = -2;
//	while(_c == -2){}
	Timer_INIT();
	initialize_platform();
	reset_pushers_to_home();
	char send[] = "data321";
	Arduino.init();
	while(1){
		printf("Hello %d \n", count);
		Arduino.sendData(send);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		count++;
		readGCode();
		update_parameters();

		prev_diffNorm = 0;
		increasing_count = 0;
		prev_SPerror = 0;
		SPerror_increasing_count = 0;

		reached = false;
		while(!reached); //waiting the process in timing.cpp

		while(_c != 0){}
		_c ++;
	}
}




