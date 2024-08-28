/*
 * PIDcontrol.cpp
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */
#include "PIDcontrol.h"
#include "mainpp.h"

const double Kp[6] = {KP_0, KP_1, KP_2, KP_3, KP_4, KP_5};
const double Ki[6] = {KI_0, KI_1, KI_2, KI_3, KI_4, KI_5};
const double Kd[6] = {KD_0, KD_1, KD_2, KD_3, KD_4, KD_5};


void calculatePID(int i) {
    pusher[i].error = pusher[i].goalVel - pusher[i].insVel;
    pusher[i].up = pusher[i].Kp * pusher[i].error;
    pusher[i].ui += pusher[i].Ki * pusher[i].last_error / frequency;
    pusher[i].ud = pusher[i].Kd * (pusher[i].error - pusher[i].last_error) * frequency;

    if (fabs(pusher[i].ui) > max_ui)
        pusher[i].ui = (pusher[i].ui >= 0) ? max_ui : -max_ui;

    pusher[i].u = pusher[i].up + pusher[i].ui + pusher[i].ud;

    pusher[i].last_error = pusher[i].error;

    pusher[i].pulse = fabs(pusher[i].u) * PWM_ARR;
    if (pusher[i].pulse > PWM_ARR) pusher[i].pulse = PWM_ARR;
}

void update_from_sensor(void) {
	for (int i = 0; i < 6; ++i) {
		pusher[i].insVel = (double)pusher[i].enc * PI * Lead
					/ (4 * resolution * reduction_ratio) * frequency;
	}
}


