/*
 * mainpp.cpp
 *
 *  Created on: Aug 22, 2024
 *      Author: sunny
 */

/*INCLUDES*/
#include "mainpp.h"

/*EXTERN VARIABLES*/
extern DMA_HandleTypeDef hdma_uart5_rx;

/*GLOBAL VARIABLES*/
char transmit_data_ptr[] = "data123";
uint8_t receive_data_ptr[100];

/*FUNCTIONS*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart5) {
		printf("%s \n", (char*) receive_data_ptr);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, receive_data_ptr,	sizeof(receive_data_ptr));
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	}
}

void main_function(void) {
	int count = 0;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, receive_data_ptr, sizeof(receive_data_ptr));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	while (1) {
		count++;
		HAL_UART_Transmit_DMA(&huart4, (uint8_t*) transmit_data_ptr, strlen(transmit_data_ptr));
		printf("Hello %d \n", count);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(500);
	}
}

