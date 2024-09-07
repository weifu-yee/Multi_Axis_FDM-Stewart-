/*
 * arduino.h
 *
 *  Created on: Sep 1, 2024
 *      Author: sunny
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

/*INCLUDE*/
#include "usart.h"
#include <stdio.h>

/*DEFINE*/
#define ARDUINO_UART_HANDLE huart4
#define ARDUINO_UART_DME_HANDLE hdma_uart4_rx

/*CLASS*/
class ARDUINO {
public:
    ARDUINO();
    virtual ~ARDUINO();
    void init();
    void sendData(char*);
private:
};

extern ARDUINO Arduino;

#endif /* ARDUINO_H_ */
