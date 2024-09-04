/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#include "mainpp.h"

//extern parameters
int cnt_5 = 0;
int t_sec = 0;
int count = 0;

bool reached = false;
Vector3D p[6], b[6];
SPPose current = create_default_stewart_platform();
SPPose next = create_default_stewart_platform();
SPPose target = create_default_stewart_platform();
double current_lengths[6], next_lengths[6];
ActuatorPID pusher[6];

double Feedrate = 0;
double amountOExtrude = 0;
double X, Y, Z, E, F, PHI, THETA, PSI;

void readGCode();
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
	while(1){
		count++;
		readGCode();
		update_parameters();
		reached = false;
		while(!reached);
	}
}




