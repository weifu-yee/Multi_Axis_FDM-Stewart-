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
#include "functions.h"

extern int count;
extern bool reached;
extern StewartPlatform current;
extern StewartPlatform target;

void main_function(void);

#ifdef __cplusplus
  }
#endif

#endif /* INC_MAINPP_H_ */
