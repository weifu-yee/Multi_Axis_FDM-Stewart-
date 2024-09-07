#ifndef _TIMING_H_
#define _TIMING_H_

#include "stm32h7xx_hal.h"
#include "main.h"

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_pusher_encoders(void);
void actuate_pushers(void);

extern bool goal;
extern double diff_lengths[6];
extern TIM_HandleTypeDef htim5;

#endif

