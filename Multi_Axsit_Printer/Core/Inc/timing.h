#ifndef _TIMING_H_
#define _TIMING_H_

#include "stm32h7xx_hal.h"
#include "main.h"

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_pusher_encoders(void);
void actuate_pushers(void);

extern bool goal;
extern double diff_lengths[7];

extern double prev_diffNorm;
extern int increasing_count;
extern double prev_SPerror;
extern int SPerror_increasing_count;

extern TIM_HandleTypeDef htim5;
//pusher1
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim13;
//pusher2
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
//pusher3
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
//pusher4
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
//pusher5
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim16;
//pusher6
extern TIM_HandleTypeDef htim24;
extern TIM_HandleTypeDef htim17;


#endif

