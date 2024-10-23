/*
 * Arduino.cpp
 *
 *  Created on: Sep 1, 2024
 *      Author: sunny
 */

#include "arduino.h"
#include "usart.h"
#include <stdio.h>
#include "stewart_platform.h"

/*GLOBAL VARIABLES*/
char transmit_data_ptr[] = "dataa12345678";
uint8_t receive_data_ptr[100] = {};
bool readFinished;

/*EXTERN VARIABLES*/
extern DMA_HandleTypeDef ARDUINO_UART_DME_HANDLE;

/*CLASS*/
class ARDUINO Arduino;

ARDUINO::ARDUINO() {}
ARDUINO::~ARDUINO() {}

void ARDUINO::init(){
	printf("arduino init\n");
	readFinished = true;
	HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
	__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
}

void ARDUINO::readGcode(void){
	char readNext[] = "next";
	Arduino.sendData(readNext);

}

void ARDUINO::sendData(char* data_){
	printf("arduino send data\n");
	readFinished = false;
	HAL_UART_Transmit_DMA(&ARDUINO_UART_HANDLE, (uint8_t*) data_, strlen(data_));
	while(!readFinished){}
}

/*OTHER FUNCTIONS*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	printf("callback\n");
	if (huart == &ARDUINO_UART_HANDLE) {
		printf("arduino receive callback: ");
		printf("%s \n", (char*) receive_data_ptr);

		if (receive_data_ptr[0] == 'G' && receive_data_ptr[1] == '1' && receive_data_ptr[2] == ' ') {
			int count = 0;
			for (int i = 3; i < (int) strlen((char*)receive_data_ptr); i++) {
				if (receive_data_ptr[i] == ';')
					break;
				count++; //num_charbers of chars including space
				if (receive_data_ptr[i] == ' ') { //i: the index of space in read[]
					char command = '\0';
					char num_char[10] = { };
					float num_float = 0.0;
					command = receive_data_ptr[i - (count - 1)];
					for (int j = 0; j < count - 2; j++) {
						num_char[j] = receive_data_ptr[i - (count - 2) + j];
					}
					for (int j = count - 2; j < 10; j++) {
						num_char[j] = '\0';
					}
					num_float = atof(num_char);
					count = 0;
					printf("command: %c\n", command);
					printf("num_char: %s\n", num_char);
					printf("num_float: %f\n", num_float);
				}
			}
		}
		X = 1.0;
		Y = 2.0;
		Z = 5.0;
		PHI = 10.0;
		THETA = 5.0;
		PSI = 0.0;
		F = 100.0;
		angularNormalizer(&PHI);
		angularNormalizer(&THETA);
		angularNormalizer(&PSI);
		HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
		__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
		for(int i = 0; i < 100; i++) receive_data_ptr[i] = 0;
	}
	update_parameters();
	readFinished = true;
}
