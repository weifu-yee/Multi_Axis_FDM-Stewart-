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

const double Kp[7] = {0, KP_1, KP_2, KP_3, KP_4, KP_5, KP_6};
const double Ki[7] = {0, KI_1, KI_2, KI_3, KI_4, KI_5, KI_6};
const double Kd[7] = {0, KD_1, KD_2, KD_3, KD_4, KD_5, KD_6};

ActuatorPID pusher[7];

extern int cnt_5;
extern int t_sec;

void reset_pushers_to_home(void) {
	//some scripts
	for (int i = 1; i <= 6; ++i) {
	   pusher[i].pulse = PWM_ARR;
	   pusher[i].u = -PWM_ARR;
	}
	cnt_5 = 0;
	t_sec = 0;
	while(t_sec < 10) {
		actuate_pushers();
	}
}

void update_pushers_PWM(const double diff_lengths[6]) {
   double max_ratio = 1.0;

   // First pass to calculate pulses and find max ratio
   for (int i = 1; i <= 6; ++i) {
       pusher[i].up = (double)Kp_univ * diff_lengths[i];
       pusher[i].u = pusher[i].up;
       pusher[i].pulse = fabs(pusher[i].u) * (double)PWM_ARR;

       if (pusher[i].pulse > PWM_ARR) {
           double ratio = (double)PWM_ARR / pusher[i].pulse;
           if (ratio < max_ratio) max_ratio = ratio;
       }
   }

   // Second pass to scale all pulses
   for (int i = 1; i <= 6; ++i) {
       pusher[i].pulse *= max_ratio;
   }
}


