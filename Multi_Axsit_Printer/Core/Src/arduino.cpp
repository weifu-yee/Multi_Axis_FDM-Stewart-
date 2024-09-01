/*
 * Arduino.cpp
 *
 *  Created on: Sep 1, 2024
 *      Author: sunny
 */

#include "arduino.h"

/*GLOBAL VARIABLES*/
char transmit_data_ptr[] = "dataa12345678";
uint8_t receive_data_ptr[100] = {};

/*EXTERN VARIABLES*/
extern DMA_HandleTypeDef ARDUINO_UART_DME_HANDLE;

/*CLASS*/
class ARDUINO Arduino;

ARDUINO::ARDUINO() {}
ARDUINO::~ARDUINO() {}

void ARDUINO::init(){
	printf("arduino init\n");
	HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
	__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
}

void ARDUINO::sendData(char* data_){
	printf("arduino send data\n");
	HAL_UART_Transmit_DMA(&ARDUINO_UART_HANDLE, (uint8_t*) data_, strlen(data_));
}

/*OTHER FUNCTIONS*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &ARDUINO_UART_HANDLE) {
		printf("arduino receive callback: ");
		printf("%s \n", (char*)receive_data_ptr);
		HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
		__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
		for(int i = 0; i < 100; i++) receive_data_ptr[i] = 0;
	}
}

