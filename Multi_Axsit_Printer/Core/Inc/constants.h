/*
 * constants.h
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

//timing
#define TOLERANCE 10
#define TREND_THRESHOLD 3  // Number of consecutive increases to detect overshoot

//stewart_platform
#define PI 3.14159265358979323846
#define H0 495
#define P_L 100
#define B_L 100
#define P_ANGLES {0, 345, 15, 105, 135, 225, 255}
#define B_ANGLES {0, 315, 45, 75, 165, 195, 285}
#define MAX_SPEED 100
#define MAX_ACCELERATION 10
#define ANG_NORM_WEIGHT 1
#define pulse_per_mm 2.27
#define SPerror_TREND_THRESHOLD 3
#define MAX_DELTA 50.0 // 每次計時器中斷時，允許的最大變化量

#define MM_Enable_GPIO_PORT_1 GPIOB
#define MM_Enable_GPIO_PIN_1 GPIO_PIN_7
#define MM_Enable_GPIO_PORT_2 GPIOE
#define MM_Enable_GPIO_PIN_2 GPIO_PIN_8

// 定義每個定時器通道和 GPIO 配置
#define ENCODER_HTIM_1 &htim1  //Encoder
#define MOTOR_HTIM_1 &htim12  //PWM
#define MOTOR_CHANNEL_1   TIM_CHANNEL_2
#define MOTOR_GPIO_PORT_1 GPIOE  //DIR
#define MOTOR_GPIO_PIN_1 GPIO_PIN_7

#define ENCODER_HTIM_2 &htim3
#define MOTOR_HTIM_2      &htim13
#define MOTOR_CHANNEL_2   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_2 GPIOE
#define MOTOR_GPIO_PIN_2 GPIO_PIN_10

#define ENCODER_HTIM_3 &htim4
#define MOTOR_HTIM_3      &htim14
#define MOTOR_CHANNEL_3   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_3 GPIOE
#define MOTOR_GPIO_PIN_3 GPIO_PIN_12

#define ENCODER_HTIM_4 &htim8
#define MOTOR_HTIM_4      &htim15
#define MOTOR_CHANNEL_4   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_4 GPIOE
#define MOTOR_GPIO_PIN_4 GPIO_PIN_15

#define ENCODER_HTIM_5 &htim23
#define MOTOR_HTIM_5      &htim16
#define MOTOR_CHANNEL_5   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_5 GPIOB
#define MOTOR_GPIO_PIN_5 GPIO_PIN_10

#define ENCODER_HTIM_6 &htim24
#define MOTOR_HTIM_6      &htim17
#define MOTOR_CHANNEL_6   TIM_CHANNEL_1
#define MOTOR_GPIO_PORT_6 GPIOB
#define MOTOR_GPIO_PIN_6 GPIO_PIN_11

//TFTransform
// 定義 word2nozzleTransform 的旋轉和平移參數
#define WORD2NOZZLE_TRANSLATION_X 0.0
#define WORD2NOZZLE_TRANSLATION_Y 0.0
#define WORD2NOZZLE_TRANSLATION_Z 600.0
#define WORD2NOZZLE_ROTATION_X_DEGREE 0.0   // 繞 X 軸旋轉的度數
#define WORD2NOZZLE_ROTATION_Y_DEGREE 0.0   // 繞 Y 軸旋轉的度數
#define WORD2NOZZLE_ROTATION_Z_DEGREE 0.0  // 繞 Z 軸旋轉的度數
// 定義 bed_surface2workpiece_originTransform 的旋轉和平移參數
#define BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_X 0.0
#define BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Y 0.0
#define BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Z 0.0
#define BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_X_DEGREE 0.0
#define BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Y_DEGREE 0.0
#define BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Z_DEGREE 0.0
// 定義 bed_joint_plane2bed_surfaceTransform 的旋轉和平移參數
#define BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_X 0.0
#define BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Y 0.0
#define BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Z 10.0
#define BED_JOINT_PLANE2BED_SURFACE_ROTATION_X_DEGREE 0.0
#define BED_JOINT_PLANE2BED_SURFACE_ROTATION_Y_DEGREE 0.0
#define BED_JOINT_PLANE2BED_SURFACE_ROTATION_Z_DEGREE 0.0

//Arduino
#define F_init 50.0

//control
#define So 181.786671
#define Lo 603.886671
#define reset_pulse 1000.0
#define length_to_slowdown 30.0

#define Ho 550
#define NUM_PUSHERS 6
#define FREQUENCY 20.0
#define resolution 512.0
#define reduction_ratio 20.8
#define max_ui 1.0
#define PWM_ARR 1000.0
#define PWM_MIN 0.0
#define Lead 20.0

#define Kp_univ 0.05
#define KP_1 0.05
#define KP_2 0.05
#define KP_3 0.05
#define KP_4 0.05
#define KP_5 0.05
#define KP_6 0.05

#define KI_1 500
#define KI_2 600
#define KI_3 550
#define KI_4 525
#define KI_5 650
#define KI_6 575

#define KD_1 200
#define KD_2 250
#define KD_3 225
#define KD_4 210
#define KD_5 275
#define KD_6 235


#endif /* INC_CONSTANTS_H_ */
