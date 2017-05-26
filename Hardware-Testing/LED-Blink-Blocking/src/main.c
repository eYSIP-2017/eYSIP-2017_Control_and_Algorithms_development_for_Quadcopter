/*
 * Project Name: LED
 * File Name: main.c
 * Created: 26-May-17 16:48:00 PM
 *
 * Mentors: Sanam Shakya, Pushkar Raj
 * Author : Heethesh Vhavle
 *
 * e-Yantra Summer Internship Program 2017
 * Project: Control and Algorithms Development for Quadcopter
 *
 * Program to blink a LED and change its delay using external interrupt.
 */

#include "main.h"

static __IO uint32_t TimingDelay;
uint8_t BlinkSpeed = 0;
RCC_ClocksTypeDef RCC_Clocks;

/**********************************
 Function name	:	main
 Functionality	:	Main function
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically
 ***********************************/
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32f10x_md.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f10x.c file
	 */

	// SysTick end of count event each 1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

	// Initialize LED2
	STM_EVAL_LEDInit(LED2);

	// Initialize User_Button on STM32NUCLEO
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	// Initiate Blink Speed variable
	BlinkSpeed = 0;

	while (1)
	{
		// Test on blink speed
		if (BlinkSpeed == 0)
		{
			// LED2 Toggle each 50ms
			STM_EVAL_LEDToggle(LED2);
			Delay(50);
		}
		else if (BlinkSpeed == 1)
		{
			// LED2 Toggle each 200ms
			STM_EVAL_LEDToggle(LED2);
			Delay(200);
		}
	}
}

/**********************************
 Function name	:	Delay
 Functionality	:	To provide delay in ms
 Arguments		:	nTime --> Time in ms
 Return Value	:	None
 Example Call	:	Delay(100)
 ***********************************/
void Delay(__IO uint32_t nTime)
{
	// Set timing delay value
	TimingDelay = nTime;

	// Wait till it becomes 0
	while (TimingDelay != 0);
}

/**********************************
 Function name	:	TimingDelay_Decrement
 Functionality	:	This is the function for SysTick_Handler
					Decrement counter till 0
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically by SysTick_Handler Exception
 ***********************************/
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		// Decrement timing delay value
		TimingDelay--;
	}
}
