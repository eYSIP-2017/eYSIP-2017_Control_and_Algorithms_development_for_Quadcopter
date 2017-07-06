/*
 * MPU9250.c
 *
 *  Created on: May 29, 2017
 *      Author: Heethesh
 */

#include "MPU9250.h"
#include "stm32f1xx_hal.h"

I2C_HandleTypeDef hi2c1;

unsigned char data[2];

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* Pinout Configuration */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void _Error_Handler(char * file, int line)
{
	while (1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	}
}

unsigned char* debug_TX(void)
{
	return (unsigned char*)data;
}

void MPU9250_Init()
{
	MX_GPIO_Init();
	MX_I2C1_Init();
}

void test_MPU()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &data, 2, 100);
	data[0] = "q";
	data[1] = "\n";
	HAL_Delay(100);
	debug_TX();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

