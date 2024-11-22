/*
 * control.cpp
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */
#include "control.h"
#include <math.h>
#include "constants.h"
#include "timing.h"
#include "stewart_platform.h"

const double Kp[7] = {0, KP_1, KP_2, KP_3, KP_4, KP_5, KP_6};
const double Ki[7] = {0, KI_1, KI_2, KI_3, KI_4, KI_5, KI_6};
const double Kd[7] = {0, KD_1, KD_2, KD_3, KD_4, KD_5, KD_6};

ActuatorPID pusher[7];

extern int cnt_5;
extern int t_sec;

extern double current_lengths[7];


int leg_un_origin = 6;
extern SPPose current;
extern bool started;
extern bool determine_KP_mode;
extern double mod_Kp[7];

void reset_pushers_to_home(void) {
	double time_points[] = {140, 100};
	//1
	for (int i = 1; i <= 6; ++i) {
		pusher[i].pulse = 1000.0;
		pusher[i].u = -1.0;
	}
	cnt_5 = 0; t_sec = 0;
	actuate_pushers();
	while (cnt_5 < time_points[0]);

	//2
	bool stick_on_origin[7];
	for (int i = 0; i <= 6; ++i) {
		stick_on_origin[i] = false;
	}
	leg_un_origin = 6;
	update_pusher_encoders();
	update_from_sensor();
	for (int i = 1; i <= 6; ++i) current_lengths[i] = 0.0;
//	cnt_5 = 0; t_sec = 0;
	while (leg_un_origin) {
		update_pusher_encoders();
		update_from_sensor();
		for (int i = 1; i <= 6; ++i) {
			pusher[i].pulse = 1000.0;
			pusher[i].u = 1.0;
			if (current_lengths[i] >= So) {
				pusher[i].pulse = 0.0;
				pusher[i].u = 1.0;
				if(stick_on_origin[i] == false)	leg_un_origin--;
				stick_on_origin[i] = true;
			}
			else if ((So - current_lengths[i]) < 10.0) {
				pusher[i].pulse = 100.0;
				pusher[i].u = 1.0;
			}
		}

		if (started) {
			leg_un_origin = 0;
		}

		actuate_pushers();
		HAL_Delay(50);
	}

	//3
	for (int i = 1; i <= 6; ++i) {
		pusher[i].pulse = 0.0;
		pusher[i].u = 1.0;
		current_lengths[i] = Lo;
	}
	cnt_5 = 0; t_sec = 0;
	actuate_pushers();

	current.z = Ho;
}
void update_pushers_PWM(const double diff_lengths[6]) {
   double max_ratio = 1.0;
   double Kp_array[7] = {0, KP_1, KP_2, KP_3, KP_4, KP_5, KP_6};

   if(determine_KP_mode) {
		for(int i = 1; i <= 6; ++i) {
			Kp_array[i] = mod_Kp[i];
		}
   }

   // First pass to calculate pulses and find max ratio
   for (int i = 1; i <= 6; ++i) {
       pusher[i].up = (double)Kp_array[i] * diff_lengths[i];
       pusher[i].u = pusher[i].up;
       pusher[i].pulse = fabs(pusher[i].u) * (double)PWM_ARR;
       if (pusher[i].u >= 0.0)	pusher[i].u = 1;
       else pusher[i].u = -1.0;

       if (pusher[i].pulse > PWM_ARR) {
           double ratio = (double)PWM_ARR / pusher[i].pulse;
           if (ratio < max_ratio) max_ratio = ratio;
       }
   }

   // Second pass to scale all pulses
   for (int i = 1; i <= 6; ++i) {
       pusher[i].pulse *= max_ratio;
       // Ensure the pulse doesn't fall below the minimum speed threshold
	   if (pusher[i].pulse < PWM_MIN) {
		   pusher[i].pulse = PWM_MIN;
	   }
   }
}


