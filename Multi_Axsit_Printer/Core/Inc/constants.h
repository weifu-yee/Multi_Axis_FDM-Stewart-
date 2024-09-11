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
#define P_ANGLES {345, 15, 105, 135, 225, 255}
#define B_ANGLES {315, 45, 75, 165, 195, 285}
#define MAX_SPEED 100
#define MAX_ACCELERATION 10
#define ANG_NORM_WEIGHT 1

// 定義每個定時器通道和 GPIO 配置
#define ENCODER_HTIM_0 &htim1
#define MOTOR_HTIM_0 &htim13
#define MOTOR_CHANNEL_0   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_0 GPIOE
#define MOTOR_GPIO_PIN_0 GPIO_PIN_7

#define ENCODER_HTIM_1 &htim3
#define MOTOR_HTIM_1      &htim14
#define MOTOR_CHANNEL_1   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_1 GPIOE
#define MOTOR_GPIO_PIN_1 GPIO_PIN_10

#define ENCODER_HTIM_2 &htim4
#define MOTOR_HTIM_2      &htim15
#define MOTOR_CHANNEL_2   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_2 GPIOE
#define MOTOR_GPIO_PIN_2 GPIO_PIN_12

#define ENCODER_HTIM_3 &htim8
#define MOTOR_HTIM_3      &htim15
#define MOTOR_CHANNEL_3   TIM_CHANNEL_2
#define MOTOR_GPIO_PORT_3 GPIOE
#define MOTOR_GPIO_PIN_3 GPIO_PIN_15

#define ENCODER_HTIM_4 &htim23
#define MOTOR_HTIM_4      &htim16
#define MOTOR_CHANNEL_4   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_4 GPIOB
#define MOTOR_GPIO_PIN_4 GPIO_PIN_10

#define ENCODER_HTIM_5 &htim24
#define MOTOR_HTIM_5      &htim17
#define MOTOR_CHANNEL_5   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_5 GPIOB
#define MOTOR_GPIO_PIN_5 GPIO_PIN_11

//control
#define NUM_PUSHERS 6
#define FREQUENCY 1000
#define resolution 512
#define reduction_ratio 20.8
#define max_ui 1
#define PWM_ARR 1000
#define Lead 20

#define KP_0 10
#define KP_1 12
#define KP_2 11
#define KP_3 10
#define KP_4 13
#define KP_5 11

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
