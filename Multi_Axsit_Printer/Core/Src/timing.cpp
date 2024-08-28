#include "timing.h"
#include "mainpp.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		cnt_5++;
		t_sec = cnt_5/20;

		//update pusher[].goalVel
		plan_velocity(current_length, target_length);
		for (int i = 0; i < 6; ++i)
			calculatePID(i);
		update_from_sensor();
		if(0){reached = true;}
	}
}
