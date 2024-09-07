/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"
#include "arduino.h"
#include "main.h"
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"

//variables

int count = 0;

bool reached = false;

double X, Y, Z, E, F, PHI, THETA, PSI;

void readGCode(void){

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
	char send[] = "data321";
	Arduino.init();
	while(1){
		printf("Hello %d \n", count);
		Arduino.sendData(send);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		count++;
		readGCode();
		update_parameters();
		reached = false;
		while(!reached);
	}
}




