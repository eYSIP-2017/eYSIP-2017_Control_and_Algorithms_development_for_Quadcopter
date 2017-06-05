/*
 * Project Name: PWM-LED-Test-01
 * File Name: main.c
 * Created: 27-May-17 22:14:00 PM
 *
 * Mentors: Sanam Shakya, Pushkar Raj
 * Author : Heethesh Vhavle
 *
 * e-Yantra Summer Internship Program 2017
 * Project: Control and Algorithms Development for Quadcopter
 *
 * Test Program to control PWM for Motors 1-6 and LEDs D1-D3 on the
 * Pluto Drone.
 *
 * Motor 1 --> PB9  - TIM4 PWM CH4
 * Motor 2 --> PB8  - TIM4 PWM CH3
 * Motor 3 --> PA1  - TIM2 PWM CH2
 * Motor 4 --> PB0  - TIM3 PWM CH3
 * Motor 5 --> PB1  - TIM3 PWM CH4
 * Motor 6 --> PA11 - TIM1 PWM CH4
 *
 * White LED D1 --> PC14
 * Red   LED D2 --> PC15
 * Blue  LED D3 --> PC13
 */

#include "main.h"
#include "stm32f1xx_hal.h"

/* Time Base Handler Structures */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Function declarations */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Variable to increase/decrease PWM value */
int pwm = 0;

/**********************************
 Function name	:	main
 Functionality	:	Main function
 Arguments		:	None
 Return Value	:	None
 Example Call	:	Called automatically
 ***********************************/
int main(void)
{
	/* Init Peripherals */
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();

	/* Start PWM Timers */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	/* Increment and Decrement PWM values from 0-1000 at steps of 50
	 * PWM Counter Period: 1000, Prescaler: 2, Frequency: ~21KHz*/
	while (1)
	{
		/* Increase PWM Values */
		for (pwm = 0; pwm <= 1000; pwm += 50) //darkest to brightest: 0-100% duty cycle
		{
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm); // Motor 1
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm); // Motor 2
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm); // Motor 3
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm); // Motor 4
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm); // Motor 5
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm); // Motor 6

			HAL_Delay(10);
		}

		HAL_Delay(400);  //hold for 400ms

		/* Decrease PWM Values */
		for (pwm = 1000; pwm >= 0; pwm -= 50) //brightest to darkest: 100-0% duty cycle
		{
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm); // Motor 1
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm); // Motor 2
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm); // Motor 3
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm); // Motor 4
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm); // Motor 5
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm); // Motor 6

			HAL_Delay(10);
		}

		HAL_Delay(400);   //hold for 400ms

		/* Toggle output LED pins*/
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle LED2 on Nucleo Board

		/* Pluto Drone LEDs */
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); // White D1
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); // Red D2
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Blue D3
	}
}

/**********************************
 Function name	:	SystemClock_Config
 Functionality	:	System Clock Configuration
 Arguments		:	None
 Return Value	:	None
 Example Call	:	SystemClock_Config()
 ***********************************/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/* Configure the Systick interrupt time */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/* Configure the Systick */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**********************************
 Function name	:	MX_TIM1_Init
 Functionality	:	Timer 1 Configuration
					PWM Channel: 4
					Count Period: 0-1000
					Prescaler: 2
					Frequency: 21.333 KHz
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MX_TIM1_Init()
 ***********************************/
static void MX_TIM1_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 2;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);
}

/**********************************
 Function name	:	MX_TIM2_Init
 Functionality	:	Timer 2 Configuration
					PWM Channel: 2
					Count Period: 0-1000
					Prescaler: 2
					Frequency: 21.333 KHz
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MX_TIM2_Init()
 ***********************************/
static void MX_TIM2_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 2;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);
}

/**********************************
 Function name	:	MX_TIM3_Init
 Functionality	:	Timer 3 Configuration
					PWM Channel: 3, 4
					Count Period: 0-1000
					Prescaler: 2
					Frequency: 21.333 KHz
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MX_TIM3_Init()
 ***********************************/
static void MX_TIM3_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 2;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);
}

/**********************************
 Function name	:	MX_TIM4_Init
 Functionality	:	Timer 4 Configuration
					PWM Channel: 3, 4
					Count Period: 0-1000
					Prescaler: 2
					Frequency: 21.333 KHz
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MX_TIM4_Init()
 ***********************************/
static void MX_TIM4_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 2;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);
}

/**********************************
 Function name	:	MX_GPIO_Init
 Functionality	:	GPIO Configuration
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MX_GPIO_Init()
 ***********************************/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**********************************
 Function name	:	_Error_Handler
 Functionality	:	This function is executed in case of error occurrence
 Arguments		:	None
 Return Value	:	None
 Example Call	:	_Error_Handler(__FILE__, __LINE__)
 ***********************************/
void _Error_Handler(char * file, int line)
{
	while (1)
	{
	}
}
