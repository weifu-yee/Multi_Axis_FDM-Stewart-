#include "timing.h"
#include "mainpp.h"

TIM_HandleTypeDef* htim_array[6] = {&htim5, &htim5, &htim5, &htim5, &htim5, &htim5};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		cnt_5++;
		t_sec = cnt_5/20;

//step 1
		update_pusher_encoders();
		update_from_sensor();
//step 2
		bool goal = same_SPPose(&current, &target);
		printf("goal: %d \n", goal);
		if (!goal)
			presume_next();
//step 3
		while(1);
		calculate_leg(&next, next_lengths);
//step 4
		double diff_lengths[6];
		calculate_diff_lengths(diff_lengths);
//step 5
		update_pushers_PWM(diff_lengths);
		actuate_pushers();
//step 6
		assignSPPose(&current, &next);
//step 7
		if(goal && calculateNorm(diff_lengths) < TOLERENCE)
			reached = true;
	}
}



void update_pusher_encoders(void) {
	for (int i = 0; i < 6; i++) {
		// 獲取計數器值並存儲
		pusher[i].enc = __HAL_TIM_GetCounter(htim_array[i]);

		// 重置計數器
		__HAL_TIM_SetCounter(htim_array[i], 0);
	}
}

void actuate_pushers(void) {
    // Pusher 0
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_0, MOTOR_CHANNEL_0, pusher[0].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN1_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN2_0, GPIO_PIN_RESET);
    if (pusher[0].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN1_0, GPIO_PIN_SET);
    else if (pusher[0].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_0, MOTOR_GPIO_PIN2_0, GPIO_PIN_SET);

    // Pusher 1
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_1, MOTOR_CHANNEL_1, pusher[1].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN1_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN2_1, GPIO_PIN_RESET);
    if (pusher[1].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN1_1, GPIO_PIN_SET);
    else if (pusher[1].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_1, MOTOR_GPIO_PIN2_1, GPIO_PIN_SET);

    // Pusher 2
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_2, MOTOR_CHANNEL_2, pusher[2].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN1_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN2_2, GPIO_PIN_RESET);
    if (pusher[2].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN1_2, GPIO_PIN_SET);
    else if (pusher[2].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_2, MOTOR_GPIO_PIN2_2, GPIO_PIN_SET);

    // Pusher 3
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_3, MOTOR_CHANNEL_3, pusher[3].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN1_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN2_3, GPIO_PIN_RESET);
    if (pusher[3].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN1_3, GPIO_PIN_SET);
    else if (pusher[3].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_3, MOTOR_GPIO_PIN2_3, GPIO_PIN_SET);

    // Pusher 4
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_4, MOTOR_CHANNEL_4, pusher[4].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN1_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN2_4, GPIO_PIN_RESET);
    if (pusher[4].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN1_4, GPIO_PIN_SET);
    else if (pusher[4].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_4, MOTOR_GPIO_PIN2_4, GPIO_PIN_SET);

    // Pusher 5
    __HAL_TIM_SET_COMPARE(MOTOR_HTIM_5, MOTOR_CHANNEL_5, pusher[5].pulse);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN1_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN2_5, GPIO_PIN_RESET);
    if (pusher[5].u > 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN1_5, GPIO_PIN_SET);
    else if (pusher[5].u < 0)
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT_5, MOTOR_GPIO_PIN2_5, GPIO_PIN_SET);
}
