/*
 * timing.c
 *
 *  Created on: Jun 19, 2017
 *      Author: Heethesh
 */

#include "stm32f1xx_hal.h"
#include "timing.h"

static __IO uint32_t Micros;

uint32_t millis()
{
	return HAL_GetTick();
}

// Not working properly, uses millis
uint32_t micros()
{
	//Micros = millis() * 1000 + 1000 - SysTick->VAL/64;
	//return Micros;
	return HAL_GetTick();
}

void delay_ms(uint32_t Delay)
{
	HAL_Delay(Delay);
}
