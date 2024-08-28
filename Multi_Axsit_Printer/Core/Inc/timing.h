#ifndef _TIMING_H_
#define _TIMING_H_

#include "stm32h7xx_hal.h"
#include "main.h"

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

extern TIM_HandleTypeDef htim5;

#endif

