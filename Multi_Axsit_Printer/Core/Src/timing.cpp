#include "timing.h"
#include "mainpp.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	double time_step = 30;
	if (htim->Instance == TIM5) {
		cnt_5++;
		t_sec = cnt_5/20;


		//		update_leg_speeds(&current, &target);//有誤
		double current_length[6], target_length[6];
	    calculate_leg(&current, p, b, current_length);
	    calculate_leg(&target, p, b, target_length);

		plan_velocity(current_length, target_length, time_step);
		for(int32_t i = 0; i < 5; i++)
			calculatePID(i);
		update_from_sensor();
		//if(1){reached = true;}
	}
}
