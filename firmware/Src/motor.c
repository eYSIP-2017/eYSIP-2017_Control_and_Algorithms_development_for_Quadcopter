/*
 * motor.c
 *
 *  Created on: Jun 17, 2017
 *      Author: Heethesh
 */

/* * * * * * * * * * * * * * * * * * * *
 *									   *
 * 			 ---> +YAW (CW)			   *
 * 			|						   *
 * 			|	 -PITCH				   *
 *									   *
 * 	  M4 (CW)	X		X	M2 (CCW)   *
 * 				 \	_  /			   *
 * 				  \|^|/				   *
 * 	 	-ROLL  	   |_|	   +ROLL	   *
 * 		  		  /   \				   *
 * 				 /     \			   *
 * 	  M3 (CCW)	X		X	M1 (CW)	   *
 *									   *
 * 				 +PITCH				   *
 * 			|						   *
 * 			|---> -YAW (CCW)		   *
 *									   *
 *									   *
 * 	 QUADCOPTER MOTOR CONFIGURATION	   *
 *									   *
 * * * * * * * * * * * * * * * * * * * */

#include "common.h"
#include "devices.h"
#include "serial.h"
#include "msp.h"
#include "motor.h"

//#define MOTOR_DEBUG

extern msp_motor msp_txf_motor;
extern msp_set_motor msp_rxf_motor;

int motor_pwm [4] = {0, 0, 0, 0};

void Motor_UpdateMSP()
{
	msp_txf_motor.motor[0] = motor_pwm[0] + 1000;
	msp_txf_motor.motor[1] = motor_pwm[1] + 1000;
	msp_txf_motor.motor[2] = motor_pwm[2] + 1000;
	msp_txf_motor.motor[3] = motor_pwm[3] + 1000;

#ifdef MOTOR_DEBUG
	serialInt(motor_pwm[0]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[1]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[2]+1000);
	serialWrite('\t');
	serialInt(motor_pwm[3]+1000);
	serialWrite('\n');
#endif
}

void Motor_UpdatePWM()
{
	Motor1_SetPWM(motor_pwm[0]); // Back Right
	Motor2_SetPWM(motor_pwm[1]); // Front Right
	Motor3_SetPWM(motor_pwm[2]); // Back Left
	Motor4_SetPWM(motor_pwm[3]); // Front Left

	Motor_UpdateMSP();
}

void Motor_StopAll()
{
	Motor1_SetPWM(0); // Back Right
	Motor2_SetPWM(0); // Front Right
	Motor3_SetPWM(0); // Back Left
	Motor4_SetPWM(0); // Front Left
}

void Motor_SetRawSpeed(int m1, int m2, int m3, int m4)
{
	motor_pwm[0] = m1;
	motor_pwm[1] = m2;
	motor_pwm[2] = m3;
	motor_pwm[3] = m4;

	Motor_UpdatePWM();
}

void Motor_SetSpeed(int m1, int m2, int m3, int m4)
{
	motor_pwm[0] = constrain(m1, 0, 1000);
	motor_pwm[1] = constrain(m2, 0, 1000);
	motor_pwm[2] = constrain(m3, 0, 1000);
	motor_pwm[3] = constrain(m4, 0, 1000);

	Motor_UpdatePWM();
}

void Motor_DistributeSpeed(float throttle, float pitch, float roll, float yaw)
{
	float M1, M2, M3, M4;

	M4 = throttle - pitch - roll + yaw - 1000; // Front Left
	M2 = throttle - pitch + roll - yaw - 1000; // Front Right
	M3 = throttle + pitch - roll - yaw - 1000; // Back Left
	M1 = throttle + pitch + roll + yaw - 1000; // Back Right

	Motor_SetSpeed((int) M1, (int) M2, (int) M3, (int) M4);
}

/**********************************
 Function name	:	MSP_SetMotor_Callback
 Functionality	:	Callback function to handle received motor data
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SetMotor_Callback()
 ***********************************/
void MSP_SetMotor_Callback()
{
	float m1, m2, m3, m4;

	m1 = constrain(msp_rxf_motor.motor[0] - 1000, 0, 1000);
	m2 = constrain(msp_rxf_motor.motor[1] - 1000, 0, 1000);
	m3 = constrain(msp_rxf_motor.motor[2] - 1000, 0, 1000);
	m4 = constrain(msp_rxf_motor.motor[3] - 1000, 0, 1000);

	Motor_SetRawSpeed(m1, m2, m3, m4);
}
