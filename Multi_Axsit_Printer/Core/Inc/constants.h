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

//PIDcontrol
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
