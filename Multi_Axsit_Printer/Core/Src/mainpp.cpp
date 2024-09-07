/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"
#include <stdio.h>
//#include "arduino.h"
#include "main.h"
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"

int count = 0;
bool reached = false;
double X, Y, Z, E, F, PHI, THETA, PSI;

void readGCode(void){
	switch(count) {
		case 1:
			X = 10.0;
			Y = 20.0;
			Z = 5.0;
			PHI = 0.0;
			THETA = 0.0;
			PSI = 0.0;
			F = 100.0;
			break;
		case 2:
			X = 15.0;
			Y = 25.0;
			Z = 10.0;
			PHI = 10.0;
			THETA = 5.0;
			PSI = 2.0;
			F = 150.0;
			break;
		case 3:
			X = 20.0;
			Y = 30.0;
			Z = 15.0;
			PHI = 20.0;
			THETA = 10.0;
			PSI = 5.0;
			F = 200.0;
			break;
		case 4:
			X = 25.0;
			Y = 35.0;
			Z = 20.0;
			PHI = 30.0;
			THETA = 15.0;
			PSI = 10.0;
			F = 250.0;
			break;
		case 5:
			X = 30.0;
			Y = 40.0;
			Z = 25.0;
			PHI = 40.0;
			THETA = 20.0;
			PSI = 15.0;
			F = 300.0;
			break;
		default:
			break;
	}
}
void update_parameters(void) {
	target.disp.x = X;
	target.disp.y = Y;
	target.disp.z = Z;
	target.disp.phi = PHI;
	target.disp.theta = THETA;
	target.disp.psi = PSI;
	double dx = X - current.disp.x;
	double dy = Y - current.disp.y;
	double dz = Z - current.disp.z;
	double total_distance = sqrt(dx*dx + dy*dy + dz*dz);
	double time = total_distance / F;
	target.velo.x = dx / time;
	target.velo.y = dy / time;
	target.velo.z = dz / time;

	double dphi = PHI - current.disp.phi;
	double dtheta = THETA - current.disp.theta;
	double dpsi = PSI - current.disp.psi;
	// Normalize angular differences to [-π, π]
	dphi = fmod(dphi + M_PI, 2*M_PI) - M_PI;
	dtheta = fmod(dtheta + M_PI, 2*M_PI) - M_PI;
	dpsi = fmod(dpsi + M_PI, 2*M_PI) - M_PI;
	target.velo.phi = dphi / time;
	target.velo.theta = dtheta / time;
	target.velo.psi = dpsi / time;
}

void main_function(void){
	initialize_platform();
	reset_pushers_to_home();
	init_lengths_array(current_lengths);
	init_lengths_array(next_lengths);
//	char send[] = "data321";
//	Arduino.init();
	while(1){
		printf("Hello %d \n", count);
//		Arduino.sendData(send);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		count++;
		readGCode();

		update_parameters();
		reached = false;
		while(!reached); //waiting the process in timing.cpp
	}
}




