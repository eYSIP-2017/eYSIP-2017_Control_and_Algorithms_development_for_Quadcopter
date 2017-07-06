/*
 * serial_new.c
 *
 *  Created on: Jun 12, 2017
 *      Author: Heethesh
 *
 * Transmit --> Blocking
 * Receive 	--> Non-Blocking (Interrupts) + FIFO
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "msp.h"
#include "circular_buffer.h"
#include <math.h>
#include <serial.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern CircularBuffer rxc;

unsigned char rx_buffer[2];
char tx_buffer[100];

int serialAvailable()
{
	return (CB_Size(&rxc));
}

void serialWrite(unsigned char ch)
{
	tx_buffer[0] = ch;
	HAL_UART_Transmit(&huart1, (uint8_t*) &tx_buffer, 1, 5);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

unsigned char serialRead()
{
	unsigned char data;
	CB_Read(&rxc, &data);
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		CB_Write(&rxc, rx_buffer[0]);
		HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void serialFlush()
{
	CB_Init(&rxc);
}

void serialBegin()
{
	serialFlush();
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
}

void serialPrint(char* data)
{
	for (int i = 0; i < strlen(data); i++)
		serialWrite(data[i]);
}

void serialInt(int val)
{
	sprintf(tx_buffer, "%i", val);
	serialPrint(tx_buffer);
}

void serialFloat(float val)
{
	// Get sign
	char *tmpSign = (val < 0) ? "-" : "";
	float tmpVal = (val < 0) ? -val : val;

	int tmpInt1 = tmpVal;                  // Get the integer (678)
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123)
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123)

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf(tx_buffer, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	serialPrint(tx_buffer);
}
