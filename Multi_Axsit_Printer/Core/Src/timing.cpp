#include "timing.h"
#include <stdio.h>
#include <stdbool.h>
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"

TIM_HandleTypeDef* htim_array[7] = {
		NULL, ENCODER_HTIM_1, ENCODER_HTIM_2, ENCODER_HTIM_3, ENCODER_HTIM_4, ENCODER_HTIM_5, ENCODER_HTIM_6};

int cnt_5 = 0;
int t_sec = 0;
extern bool reached;

int a = 0;

void update_pusher_encoders(void) {
	for (int i = 1; i <= 6; i++) {
		// 獲取計數器值並存儲
		pusher[i].enc = __HAL_TIM_GetCounter(htim_array[i]);

		// 重置計數器
		__HAL_TIM_SetCounter(htim_array[i], 0);
	}
}

void actuate_pushers(void) {
	a ++;
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_1, MOTOR_CHANNEL_1, pusher[1].pulse);
    if (pusher[1].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN_1, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_2, MOTOR_CHANNEL_2, pusher[2].pulse);
    if (pusher[2].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN_2, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_3, MOTOR_CHANNEL_3, pusher[3].pulse);
    if (pusher[3].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_4, MOTOR_CHANNEL_4, pusher[4].pulse);
    if (pusher[4].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN_4, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN_4, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_5, MOTOR_CHANNEL_5, pusher[5].pulse);
    if (pusher[5].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN_5, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_6, MOTOR_CHANNEL_6, pusher[6].pulse);
    if (pusher[6].dir > 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_6, MOTOR_GPIO_PIN_6, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_6, MOTOR_GPIO_PIN_6, GPIO_PIN_RESET);
}

extern int line_of_Gcode;
bool dir = 1;
int pwm = 100;
bool goal = false;
double diff_lengths[7];

int _c = 1;
int bbb = 0;
double diffNorm = 0;
extern int S;
extern bool stop;
extern bool started;

double prev_diffNorm = 0;
int increasing_count = 0;


double PWM[7] = {0, 0, 0, 0, 0, 0};

/*goal: same Pose ; reached: nearly reached lengths */

void true_process(void) {
//step 1
		update_pusher_encoders();
		update_from_sensor();
//		fake_update_from_sensor();
//step 2
		goal = same_SPPose(&current, &target);
		if (!goal) {
			presume_next();
		}
//step 3
		calculate_leg(&next, next_lengths);
//step 4
		calculate_diff_lengths(diff_lengths);
//step 5
		update_pushers_PWM(diff_lengths);
		actuate_pushers();
//step 6
		assignSPPose(&current, &next);  //IMU
//step 7
		diffNorm = calculateNorm(diff_lengths);

		// Detect if we're getting further away from the target
		if (diffNorm > prev_diffNorm) {
			increasing_count++;
		} else {
			increasing_count = 0;
		}

		if ((goal && diffNorm < TOLERANCE) ||
			(goal && increasing_count >= TREND_THRESHOLD)) {
			reached = true;
		}

		prev_diffNorm = diffNorm;
}
void test_process(void) {
//for test
		update_pusher_encoders();
		update_from_sensor();
		for (int i = 1; i <= 6; ++i) {
			pusher[i].pulse = fabs(PWM[i]);
			if(PWM[i] >= 0.0) pusher[i].dir = 1.0;
			else pusher[i].dir = -1.0;
		}
		if (t_sec > 2) {
			for (int i = 1; i <= 6; ++i) {
				pusher[i].dir = 0.0;
				pusher[i].pulse = 0.0;
			}
		}
		actuate_pushers();
}
void one_leg_process(int leg) {
//step 1
		update_pusher_encoders();
		update_from_sensor();
//step 2
		goal = same_SPPose(&current, &target);
		if (!goal) {
			presume_next();
		}
//step 3
		calculate_leg(&next, next_lengths);
//step 4
		calculate_diff_lengths(diff_lengths);

		for(int i = 1; i <= 6; ++i) {
			if(i == leg) continue;
			diff_lengths[i] = 0.0;
		}
//step 5
		update_pushers_PWM(diff_lengths);
		actuate_pushers();
//step 6
		assignSPPose(&current, &next);  //IMU
//step 7
		diffNorm = calculateNorm(diff_lengths);

		// Detect if we're getting further away from the target
		if (diffNorm > prev_diffNorm) {
			increasing_count++;
		} else {
			increasing_count = 0;
		}

		if ((goal && diffNorm < TOLERANCE) ||
			(goal && increasing_count >= TREND_THRESHOLD)) {
			reached = true;
		}

		prev_diffNorm = diffNorm;
}
void three_leg_process(int leg1, int leg2, int leg3) {
//step 1
	update_pusher_encoders();
	update_from_sensor();
//step 2
	goal = same_SPPose(&current, &target);
	if (!goal) {
		presume_next();
	}
//step 3
	calculate_leg(&next, next_lengths);
//step 4
	calculate_diff_lengths(diff_lengths);

	for(int i = 1; i <= 6; ++i) {
		if(i == leg1) continue;
		else if(i == leg2) continue;
		else if(i == leg3) continue;
		diff_lengths[i] = 0.0;
	}
//step 5
	update_pushers_PWM(diff_lengths);
	actuate_pushers();
//step 6
	assignSPPose(&current, &next);  //IMU
//step 7
	diffNorm = calculateNorm(diff_lengths);

	// Detect if we're getting further away from the target
	if (diffNorm > prev_diffNorm) {
		increasing_count++;
	} else {
		increasing_count = 0;
	}

	if ((goal && diffNorm < TOLERANCE) ||
		(goal && increasing_count >= TREND_THRESHOLD)) {
		reached = true;
	}

	prev_diffNorm = diffNorm;
}
void fake_encoder_process(void) {
//step 1
		update_pusher_encoders();
		fake_update_from_sensor();
//step 2
		goal = same_SPPose(&current, &target);
		if (!goal) {
			presume_next();
		}
//step 3
		calculate_leg(&next, next_lengths);
//step 4
		calculate_diff_lengths(diff_lengths);
//step 5
		update_pushers_PWM(diff_lengths);
		actuate_pushers();
//step 6
		assignSPPose(&current, &next);  //IMU
//step 7
		diffNorm = calculateNorm(diff_lengths);

		// Detect if we're getting further away from the target
		if (diffNorm > prev_diffNorm) {
			increasing_count++;
		} else {
			increasing_count = 0;
		}

		if ((goal && diffNorm < TOLERANCE) ||
			(goal && increasing_count >= TREND_THRESHOLD)) {
			reached = true;
		}

		prev_diffNorm = diffNorm;
}
void determine_KP_process(void) {
	//draw a rectangular on a plane that z axis is fixed.

//step 1
		update_pusher_encoders();
		update_from_sensor();
//step 2
		goal = same_SPPose(&current, &target);
		if (!goal) {
			presume_next();
		}
//step 3
		calculate_leg(&next, next_lengths);
//step 4
		calculate_diff_lengths(diff_lengths);
//step 5
		update_pushers_PWM(diff_lengths);
		actuate_pushers();
//step 6
		assignSPPose(&current, &next);  //IMU
//step 7
		diffNorm = calculateNorm(diff_lengths);

		// Detect if we're getting further away from the target
		if (diffNorm > prev_diffNorm) {
			increasing_count++;
		} else {
			increasing_count = 0;
		}

		if ((goal && diffNorm < TOLERANCE) ||
			(goal && increasing_count >= TREND_THRESHOLD)) {
			reached = true;
		}

		prev_diffNorm = diffNorm;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		cnt_5++; t_sec = (int)cnt_5/FREQUENCY;

		if (stop && started) {
			//once press the ArduinoRest button, and press start
			started = false;
			stop = false;
			line_of_Gcode = 0;
		}

		if (reached) {
			if(line_of_Gcode) {
				for (int i = 1; i <= 6; ++i) { //wait for start button be clicked
					pusher[i].u = 0.0;
					pusher[i].pulse = 0.0;
				}
				actuate_pushers();
			}
			//_c is for debug.
			_c ++;
			return;
		}

		if (S) {
			for (int i = 1; i <= 6; ++i) { //wait for start button be clicked
				pusher[i].u = 0.0;
				pusher[i].pulse = 0.0;
			}
			actuate_pushers();
			--S;
			return;
		}

		//choose one process
		int proc = 1;
		//

		switch(proc) {
			case 1:
				true_process();
				break;
			case 2:
				test_process();
				break;
			case 3:
				one_leg_process(2); //4th leg
				break;
			case 4:
				three_leg_process(4, 5, 6);
				break;
			case 5:
				fake_encoder_process();
				break;
			default:
				break;
		}
	}
}
