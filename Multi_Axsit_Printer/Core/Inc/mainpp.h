/*
 * mainpp.h
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB

#ifdef __cplusplus
  extern "C"{
#endif

void main_function(void);

#ifdef __cplusplus
  }
#endif

#endif /* INC_MAINPP_H_ */
