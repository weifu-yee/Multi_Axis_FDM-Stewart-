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
#include "TFTransform.h"
#include "constants.h"

/*GLOBAL VARIABLES*/
uint8_t receive_data_ptr[100] = {};
bool readFinished, readAnotherLine;
double F;
double X, Y, Z, A, B, C; //G1
double w2p_X, w2p_Y, w2p_Z, w2p_A, w2p_B, w2p_C; //G87
int S; //G4
bool stop;

/*EXTERN VARIABLES*/
extern DMA_HandleTypeDef ARDUINO_UART_DME_HANDLE;

/*CLASS*/
class ARDUINO Arduino;

ARDUINO::ARDUINO() {}
ARDUINO::~ARDUINO() {}

void ARDUINO::init(){
	printf("arduino init\n");
	readFinished = true;
	readAnotherLine = false;
	F = F_init;
	X = 0; Y = 0; Z = 0; A = 0; B = 0; C = 0;
	w2p_X = 0; w2p_Y = 0; w2p_Z = 0; w2p_A = 0; w2p_B = 0; w2p_C = 0;
	S = 0;
	stop = false;
	HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
	__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
}

void ARDUINO::readGcode(void){
	char readNext[] = "next";
	do{
		sendData(readNext);
		while(!readFinished){}
	} while (readAnotherLine);
}

void ARDUINO::sendData(char* data_){
	readFinished = false;
	readAnotherLine = true;
	HAL_UART_Transmit_DMA(&ARDUINO_UART_HANDLE, (uint8_t*) data_, strlen(data_));
}

/*OTHER FUNCTIONS*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &ARDUINO_UART_HANDLE) {
		printf("arduino receive callback: ");
		printf("%s \n", (char*) receive_data_ptr);

		if (receive_data_ptr[0] == 'G' && receive_data_ptr[1] == '1' && receive_data_ptr[2] == ' ') {
			int count = 0;
			for (int i = 3; i < (int) strlen((char*)receive_data_ptr); i++) {
				if (receive_data_ptr[i] == ';')
					break;
				count++; //numbers of chars including space
				if (receive_data_ptr[i] == ' ') { //i: the index of space in read[]
					char command = '\0';
					char num_char[10] = { };
					double num_double = 0.0;
					command = receive_data_ptr[i - (count - 1)];
					for (int j = 0; j < count - 2; j++) {
						num_char[j] = receive_data_ptr[i - (count - 2) + j];
					}
					for (int j = count - 2; j < 10; j++) {
						num_char[j] = '\0';
					}
					num_double = atof(num_char);
					count = 0;
					// printf("command: %c\n", command);
					// printf("num_char: %s\n", num_char);
					// printf("num_double: %f\n", num_double);
					if(command == 'X') X = num_double;
					if(command == 'Y') Y = num_double;
					if(command == 'Z') Z = num_double;
					if(command == 'A') A = num_double;
					if(command == 'B') B = num_double;
					if(command == 'C') C = num_double;
					if(command == 'F') F = num_double;
				}
			}
			angularNormalizer(&A);
			angularNormalizer(&B);
			angularNormalizer(&C);
			transformer.setPartToNozzleTransform(X, Y, Z, A, B, C);
			readAnotherLine = false;
		}
		else if (receive_data_ptr[0] == 'G' && receive_data_ptr[1] == '8'
				&& receive_data_ptr[2] == '7' && receive_data_ptr[3] == ' ') {
			int count = 0;
			for (int i = 4; i < (int) strlen((char*) receive_data_ptr); i++) {
				if (receive_data_ptr[i] == ';') break;
				count++; //numbers of chars including space
				if (receive_data_ptr[i] == ' ') { //i: the index of space in read[]
					char command = '\0';
					char num_char[10] = { };
					double num_double = 0.0;
					command = receive_data_ptr[i - (count - 1)];
					for (int j = 0; j < count - 2; j++) {
						num_char[j] = receive_data_ptr[i - (count - 2) + j];
					}
					for (int j = count - 2; j < 10; j++) {
						num_char[j] = '\0';
					}
					num_double = atof(num_char);
					count = 0;
					//printf("command: %c\n", command);
					//printf("num_char: %s\n", num_char);
					//printf("num_double: %f\n", num_double);
					if (command == 'X') w2p_X = num_double;
					if (command == 'Y') w2p_Y = num_double;
					if (command == 'Z') w2p_Z = num_double;
					if (command == 'A') w2p_X = num_double;
					if (command == 'B') w2p_Y = num_double;
					if (command == 'C') w2p_Z = num_double;
				}
			}
			transformer.setWorkpieceOriginToPartTransform(w2p_X, w2p_Y, w2p_Z, w2p_A, w2p_B, w2p_C);
		}
		else if (receive_data_ptr[0] == 'G' && receive_data_ptr[1] == '4' && receive_data_ptr[2] == ' ') {
			int count = 0;
			for (int i = 3; i < (int) strlen((char*) receive_data_ptr); i++) {
				if (receive_data_ptr[i] == ';') break;
				count++; //numbers of chars including space
				if (receive_data_ptr[i] == ' ') { //i: the index of space in read[]
					char command = '\0';
					char num_char[10] = { };
					int num_int = 0;
					command = receive_data_ptr[i - (count - 1)];
					for (int j = 0; j < count - 2; j++) {
						num_char[j] = receive_data_ptr[i - (count - 2) + j];
					}
					for (int j = count - 2; j < 10; j++) {
						num_char[j] = '\0';
					}
					num_int = (int)atof(num_char);
					count = 0;
					//printf("command: %c\n", command);
					//printf("num_char: %s\n", num_char);
					//printf("num_double: %f\n", num_double);
					if (command == 'S') S = num_int * FREQUENCY;
				}
			}
		}else if (receive_data_ptr[0] == 'M' && receive_data_ptr[1] == '2' && receive_data_ptr[2] == ' ') {
			stop = true;
		}
		
		SPPose pose = transformer.getJointPlanePoseInWorldFrame();
		update_parameters(&pose, F);
		for(int i = 0; i < 100; i++) receive_data_ptr[i] = 0;
		readFinished = true;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&ARDUINO_UART_HANDLE, receive_data_ptr, 100);
	__HAL_DMA_DISABLE_IT(&ARDUINO_UART_DME_HANDLE, DMA_IT_HT);
}
