/*
 * constants.h
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

//timing
#define TOLERENCE 1

//stewart_platform
#define PI 3.14159265358979323846
#define H0 495000
#define P_L 100000
#define B_L 100000
#define P_ANGLES {350, 10, 110, 130, 230, 250}
#define B_ANGLES {310, 50, 70, 170, 190, 290}
#define MAX_SPEED 100
#define MAX_ACCELERATION 10
#define ANG_NORM_WEIGHT 1

// 定義每個定時器通道和 GPIO 配置
#define MOTOR_HTIM_0 &htim5
#define MOTOR_CHANNEL_0   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_0 GPIOA
#define MOTOR_GPIO_PIN1_0 GPIO_PIN_0
#define MOTOR_GPIO_PIN2_0 GPIO_PIN_1

#define MOTOR_HTIM_1      &htim5
#define MOTOR_CHANNEL_1   TIM_CHANNEL_2
#define MOTOR_GPIO_PORT_1 GPIOB
#define MOTOR_GPIO_PIN1_1 GPIO_PIN_2
#define MOTOR_GPIO_PIN2_1 GPIO_PIN_3

#define MOTOR_HTIM_2      &htim5
#define MOTOR_CHANNEL_2   TIM_CHANNEL_3
#define MOTOR_GPIO_PORT_2 GPIOC
#define MOTOR_GPIO_PIN1_2 GPIO_PIN_4
#define MOTOR_GPIO_PIN2_2 GPIO_PIN_5

#define MOTOR_HTIM_3      &htim5
#define MOTOR_CHANNEL_3   TIM_CHANNEL_4
#define MOTOR_GPIO_PORT_3 GPIOD
#define MOTOR_GPIO_PIN1_3 GPIO_PIN_6
#define MOTOR_GPIO_PIN2_3 GPIO_PIN_7

#define MOTOR_HTIM_4      &htim5
#define MOTOR_CHANNEL_4   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_4 GPIOE
#define MOTOR_GPIO_PIN1_4 GPIO_PIN_8
#define MOTOR_GPIO_PIN2_4 GPIO_PIN_9

#define MOTOR_HTIM_5      &htim5
#define MOTOR_CHANNEL_5   TIM_CHANNEL_2
#define MOTOR_GPIO_PORT_5 GPIOB
#define MOTOR_GPIO_PIN1_5 GPIO_PIN_10
#define MOTOR_GPIO_PIN2_5 GPIO_PIN_11

//control
#define NUM_PUSHERS 6
#define FREQUENCY 1000
#define resolution 512
#define reduction_ratio 20.8
#define max_ui 1
#define PWM_ARR 4200
#define Lead 20

#define KP_0 1000
#define KP_1 1200
#define KP_2 1100
#define KP_3 1050
#define KP_4 1300
#define KP_5 1150

#define KI_0 500
#define KI_1 600
#define KI_2 550
#define KI_3 525
#define KI_4 650
#define KI_5 575

#define KD_0 200
#define KD_1 250
#define KD_2 225
#define KD_3 210
#define KD_4 275
#define KD_5 235


#endif /* INC_CONSTANTS_H_ */
