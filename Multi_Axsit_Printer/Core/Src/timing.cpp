#include "timing.h"
#include <stdio.h>
#include <stdbool.h>
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"

TIM_HandleTypeDef* htim_array[6] = {
		ENCODER_HTIM_0, ENCODER_HTIM_1, ENCODER_HTIM_2, ENCODER_HTIM_3, ENCODER_HTIM_4, ENCODER_HTIM_5};

int cnt_5 = 0;
int t_sec = 0;
extern bool reached;


void update_pusher_encoders(void) {
	for (int i = 0; i < 6; i++) {
		// 獲取計數器值並存儲
		pusher[i].enc = __HAL_TIM_GetCounter(htim_array[i]);

		// 重置計數器
		__HAL_TIM_SetCounter(htim_array[i], 0);
	}
}

void actuate_pushers(void) {
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_0, MOTOR_CHANNEL_0, pusher[0].pulse);
    if (pusher[0].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN_0, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN_0, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_1, MOTOR_CHANNEL_1, pusher[1].pulse);
    if (pusher[1].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN_1, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_2, MOTOR_CHANNEL_2, pusher[2].pulse);
    if (pusher[2].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN_2, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_3, MOTOR_CHANNEL_3, pusher[3].pulse);
    if (pusher[3].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_4, MOTOR_CHANNEL_4, pusher[4].pulse);
    if (pusher[4].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN_4, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN_4, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_5, MOTOR_CHANNEL_5, pusher[5].pulse);
    if (pusher[5].u >= 0)        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN_5, GPIO_PIN_SET);
    else        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN_5, GPIO_PIN_RESET);
}

extern int count;
bool dir = 1;
int pwm = 1000;
bool goal = false;
double diff_lengths[6];

int _c = 1;
int bbb = 0;
double diffNorm = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	_c = 300;
	if (htim->Instance == TIM5) {
		if (reached)	{
			_c ++;
			return;
		}
		cnt_5++;
		t_sec = cnt_5/20;


//step 1
		update_pusher_encoders();
		//update_from_sensor();
		fake_update_from_sensor();
//step 2
		goal = same_SPPose(&current, &target);
		if (!goal) {
			presume_next();
			//step 3
			calculate_leg(&next, next_lengths);
			//step 4
			calculate_diff_lengths(diff_lengths);
			//step 5
			update_pushers_PWM(diff_lengths);
			actuate_pushers();
			//step 6
			assignSPPose(&current, &next);  //IMU
		}
//step 7
		diffNorm = calculateNorm(diff_lengths);
		if(goal && diffNorm < TOLERENCE)
			reached = true;




		if(!reached) {
			bbb = 2;
			while(_c == 1);
			_c = 1;
		}



		//測試腳位輸出用
//		update_pusher_encoders();
//		update_from_sensor();
//
//		__HAL_TIM_SET_COMPARE(MOTOR_HTIM_3, MOTOR_CHANNEL_3, pwm);
//		HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_SET);
//		if(!dir)
//			HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN_3, GPIO_PIN_RESET);
//
////		int a = t_sec / 3;
////		if(a % 2 == 0)
////			HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN_0, GPIO_PIN_SET);
////		else
////			HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN_0, GPIO_PIN_RESET);
//
//		if (t_sec > 2)
//			__HAL_TIM_SET_COMPARE(MOTOR_HTIM_3, MOTOR_CHANNEL_3, 0);


	}
}
