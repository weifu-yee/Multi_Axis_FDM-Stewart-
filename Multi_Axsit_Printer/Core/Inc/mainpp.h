/*
 * mainpp.h
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#ifdef __cplusplus
  extern "C"{
#endif

#include <stdio.h>
#include "main.h"
#include "stewart_platform.h"
#include "constants.h"
#include "control.h"
#include <stdbool.h>

//extern variables
extern int cnt_5;
extern int t_sec;
extern int count;

extern bool reached;
extern Vector3D p[6];
extern Vector3D b[6];
extern SPPose current;
extern SPPose next;
extern SPPose target;
extern double current_lengths[6];
extern double next_lengths[6];
extern ActuatorPID pusher[6];


void main_function(void);

#ifdef __cplusplus
  }
#endif

#endif /* INC_MAINPP_H_ */
