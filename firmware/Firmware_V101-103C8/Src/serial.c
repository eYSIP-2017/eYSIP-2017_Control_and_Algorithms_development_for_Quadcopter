/*
 * uart.c
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

/* TODO: Debug serial RX functions */

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "serial.h"
#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
unsigned char buffer_rx[100];
char buffer_tx[100];

void serialWrite(unsigned char chr)
{
	buffer_tx[0] = chr;
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, 1, 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void serialWriteBytes(unsigned char* data, unsigned int len)
{
	sprintf(buffer_tx, "%s", data);
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, strlen(buffer_tx), 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void serialLine(void)
{
	buffer_tx[0] = '\n';
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, 1, 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void serialPrint(char* str)
{
	sprintf(buffer_tx, "%s", str);
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, strlen(buffer_tx), 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void serialInt(int val)
{
	sprintf(buffer_tx, "%i", val);
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, strlen(buffer_tx), 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void serialFloat(float val)
{
	// Get sign
	char *tmpSign = (val < 0) ? "-" : "";
	float tmpVal  = (val < 0) ? -val : val;

	int tmpInt1 = tmpVal;                  // Get the integer (678)
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123)
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123)

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf(buffer_tx, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	HAL_UART_Transmit(&huart1, (uint8_t*) &buffer_tx, strlen(buffer_tx), 50);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

unsigned char serialRead()
{
	unsigned char data;
	uint8_t ret = HAL_UART_Receive(&huart1, buffer_rx, 1, 50);
	if (ret != HAL_OK) return 0;

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	data = buffer_rx[0];
	return data;
}

unsigned char serialReadBytes(unsigned char* data, unsigned int len)
{
	if (HAL_UART_Receive(&huart1, &data, len, 50) != HAL_OK)
		return 0;

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	return 1;
}
