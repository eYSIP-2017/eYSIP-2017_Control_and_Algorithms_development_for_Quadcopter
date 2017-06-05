/*
 * Project Name: LED
 * File Name: main.c
 * Created: 26-May-17 19:58:00 PM
 *
 * Mentors: Sanam Shakya, Pushkar Raj
 * Author : Heethesh Vhavle
 *
 * e-Yantra Summer Internship Program 2017
 * Project: Control and Algorithms Development for Quadcopter
 *
 * Program to blink a LED using a Task Scheduler
 */

#include "main.h"

static __IO uint32_t time_ms = 0;
static __IO uint32_t time_sec = 0;
uint32_t last_task_time = 0;
uint32_t BlinkSpeed = 0;
RCC_ClocksTypeDef RCC_Clocks;

/**********************************
 Function name	:	TimerHandler
 Functionality	:	To handle the time in ms
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically by SysTick_Handler()
 ***********************************/
void TimerHandler()
{
	time_ms++;
	if (time_ms >= 1000)
	{
		// Restore value
		time_ms = 0;
		time_sec++;
	}
}

/**********************************
 Function name	:	millis
 Functionality	:	To keep program time from the start
 Arguments		:	None
 Return Value	:	Current program time in ms
 Example Call	:	millis()
 ***********************************/
uint32_t millis()
{
	return ((time_sec * 1000) + time_ms);
}

/**********************************
 Function name	:	taskScheduler
 Functionality	:	To schedule different tasks using SysTick
 Arguments		:	None
 Return Value	:	None
 Example Call	:	taskScheduler()
 ***********************************/
void taskScheduler()
{
	if ((millis() - last_task_time) > BlinkSpeed)
	{
		// Store current time
		last_task_time = millis();

		// Toggle LED
		STM_EVAL_LEDToggle(LED2);
	}
}

/**********************************
 Function name	:	setup
 Functionality	:	To setup and initiate all devices
 Arguments		:	None
 Return Value	:	None
 Example Call	:	setup()
 ***********************************/
void setup()
{
	// SysTick end of count event each 1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

	// Initialize LED2
	STM_EVAL_LEDInit(LED2);

	// Initialize User_Button on STM32NUCLEO
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	// Initiate Blink Speed variable
	BlinkSpeed = 50;
}

/**********************************
 Function name	:	main
 Functionality	:	Main function
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically
 ***********************************/
int main(void)
{
	setup();

	// Run task scheduler continuously
	while (1) taskScheduler();
}
