/*
 * uart.c
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "serial.h"
#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

char buffer_tx[100];

/*int intLength(int i)
 {
 int len = 0;
 if (i < 0) len++;
 for(; i; i /= 10) len++;
 return (len == 0) ? 1 : len;
 }*/

void printChar(unsigned char chr)
{
	buffer_tx[0] = chr;
	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer_tx, 1, 200); // Using Interrupts
	//HAL_UART_Transmit(&huart2, (uint8_t*) buffer_tx, 1, 100); // Without Interrupt
	//HAL_Delay(10); // Remove delay if using UART without interrupts
}

void printLine(void)
{
	buffer_tx[0] = '\n';
	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer_tx, 1, 100);
	//HAL_Delay(10);
}

void printString(char* str)
{
	sprintf(buffer_tx, "%s", str);
	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer_tx, strlen(buffer_tx), 100);
	//HAL_Delay(10);
}

void printInt(int val)
{
	sprintf(buffer_tx, "%i", val);
	HAL_UART_Transmit(&huart2, (uint8_t*) &buffer_tx, strlen(buffer_tx), 100);
	//HAL_Delay(10);
}

void printFloat(float val)
{
	char *tmpSign = (val < 0) ? "-" : "";
	float tmpVal  = (val < 0) ? -val : val;

	int tmpInt1 = tmpVal;                  // Get the integer (678)
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123)
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123)

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf(buffer_tx, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer_tx, strlen(buffer_tx), 100);
	//HAL_Delay(10);
}

