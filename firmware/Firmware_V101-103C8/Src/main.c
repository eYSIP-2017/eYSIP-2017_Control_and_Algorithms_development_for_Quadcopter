/*
 * Project Name: Firmware V101-103C8
 * File Name: main.c
 * Created: June 7, 2017
 *
 * Mentors: Sanam Shakya, Pushkar Raj
 * Author : Heethesh Vhavle
 *
 * e-Yantra Summer Internship Program 2017
 * Project: Control and Algorithms Development for Quadcopter
 *
 * Firmware for Pluto Drone
 * Version: 1.0.1A
 * Target: STM32F103C8
 *
 * Hardware Connections:
 *
 * I2C1 Bus --> 0x68 - MPU9250
 * I2C1 Bus --> 0x0C - AK8963
 * I2C1 Bus --> 0x-- - MX5611
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
 *
 * ESP8266 --> USART1 TX/RX/BOOT0/RESET
 */

/* TODO: Document all libraries */
/* TODO: Implement MSP RX functions */
/* TODO: Calibrate Pluto Drone IMU */
/* TODO: Interface barometer and voltage sensor */

#include "devices.h"
#include "stm32f1xx_hal.h"
#include "serial.h"
#include "MPU9250.h"
#include "telemetry.h"
#include "msp.h"
#include "main.h"

void setup(void)
{
	Devices_Init();
	//IMU_Init();
}

int main(void)
{
	setup();

	/** PWM Test Code (PWM 0-1000, 21.333 KHz)*/
	/*Motor1_SetPWM(300);
	Motor2_SetPWM(300);
	Motor3_SetPWM(300);
	Motor4_SetPWM(300);*/

	while (1)
	{
		/** IMU Test Code */
		//AHRS_ComputeAngles();

		/** MSP Transmit Test Code */
		/*msp_txf_attitude.angx = i*10;
		msp_txf_attitude.angy = i*5;
		msp_txf_attitude.heading = i;
		MSP_SendAttitude();

		if (i>180) {j=0;}
		else if (i<-180) {j=1;}

		if (j) i++;
		else i--;*/

		/** MSP Echo Test */
		/*if (MSP_RecieveRC())
		{
			msp_txf_rc.roll = msp_rxf_raw_rc.roll;
			msp_txf_rc.pitch = msp_rxf_raw_rc.pitch;
			msp_txf_rc.yaw = msp_rxf_raw_rc.yaw;
			msp_txf_rc.throttle = msp_rxf_raw_rc.throttle;
			msp_txf_rc.aux1 = msp_rxf_raw_rc.aux1;
			msp_txf_rc.aux2 = msp_rxf_raw_rc.aux2;
			msp_txf_rc.aux3 = msp_rxf_raw_rc.aux3;
			msp_txf_rc.aux4 = msp_rxf_raw_rc.aux4;
			MSP_SendRC();
		}*/

		/** UART Hello World */
		serialPrint("Hello!\n");
		HAL_Delay(100);
	}
}
