/*
 * pid.c
 *
 *  Created on: June 15, 2017
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

#include "stm32f1xx_hal.h"
#include "peripherals.h"
#include "devices.h"
#include "common.h"
#include "MPU9250.h"
#include "msp.h"
#include "serial.h"
#include "timing.h"
#include "motor.h"
#include "pid.h"
#include <string.h>

//#define PID_DEBUG
#define PID_LIMIT 600

PID_TypeDef pid_pitch = {PITCH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_roll = {ROLL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_yaw = {YAW, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PID_TypeDef pid_altitude = {ALT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

extern msp_set_raw_rc msp_rxf_raw_rc;
extern msp_set_pid msp_rxf_pid;
uint8_t MOTOR_ARM = 0;

typedef struct __attribute__((__packed__))
{
	float set_point;
	float kp;
	float ki;
	float kd;
}Debug_Buffer;

Debug_Buffer dbuff;
uint8_t debug_buffer[16];

void MSP_SetPID_Callback()
{
	/*pid_pitch.con_KP = msp_rxf_pid.pitch.p/10.0;
	pid_pitch.con_KI = msp_rxf_pid.pitch.i/200.0;
	pid_pitch.con_KD = msp_rxf_pid.pitch.d*4.0;

	pid_roll.con_KP = msp_rxf_pid.roll.p/10.0;
	pid_roll.con_KI = msp_rxf_pid.roll.i/200.0;
	pid_roll.con_KD = msp_rxf_pid.roll.d*4.0;

	pid_yaw.con_KP = msp_rxf_pid.yaw.p;
	pid_yaw.con_KI = msp_rxf_pid.yaw.i;
	pid_yaw.con_KD = msp_rxf_pid.yaw.d;

	pid_altitude.con_KP = msp_rxf_pid.alt.p;
	pid_altitude.con_KI = msp_rxf_pid.alt.i;
	pid_altitude.con_KD = msp_rxf_pid.alt.d;

	dbuff.set_point = pid_pitch.set_point;
	dbuff.kp = pid_pitch.con_KP;
	dbuff.ki = pid_pitch.con_KI;
	dbuff.kd = pid_pitch.con_KD;

	memcpy(debug_buffer, &dbuff, 16);
	for (int i=0; i<16; i++)
		serialWrite(debug_buffer[i]);*/
}

void MSP_SetRawRC_Callback()
{
	if (msp_rxf_raw_rc.aux1 > 1600) MOTOR_ARM = 1;
	else MOTOR_ARM = 0;

	pid_altitude.output = msp_rxf_raw_rc.throttle;
	PID_SetPoint(0, msp_rxf_raw_rc.pitch, msp_rxf_raw_rc.roll, msp_rxf_raw_rc.yaw);
}

void PID_Compute(PID_TypeDef *pid)
{
	float KP = 0, KI = 0, KD = 0;

	// Check loop time
	pid->time = micros();
	if ((micros() - pid->last_time) <= pid->delta) return;

	// Compute error
	pid->error = (pid->set_point + pid->offset) - pid->input;

	/* --------------------------- YAW WRAP ERROR ------------------------------
     * An example in degrees is the best way to explain this.  If the target is
     * -179 degrees and the input is +179 degrees, the standard PID output would
     * be -358 degrees leading to a very high yaw rotation rate to correct the
     * -358 degrees error.  However, +2 degrees achieves the same result, with
     * a  much lower rotation rate to fix the error.
     * -----------------------------------------------------------------------*/
	if (pid->instance == YAW)
	{
		if (abs(pid->error) > 180)
		pid->error = pid->error - (360 * pid->error / abs(pid->error));
	}

	if (pid_altitude.output < pid->breakpoint)
	{
		KP = pid->con_KP;
		KI = pid->con_KI;
		KD = pid->con_KD;
	}
	else
	{
		KP = pid->agr_KP;
		KI = pid->agr_KI;
		KD = pid->agr_KD;
	}

	// Compute proportional term
	pid->proportional = KP * pid->error;

	// Compute integral sum
	pid->integral += pid->error;

	// Constrain integral term to prevent wind-up
	pid->integral = constrain(pid->integral, -PID_LIMIT, PID_LIMIT);

	// Compute derivative term
	pid->derivative = pid->input - pid->last_input;

	// Compute angle PID output
	pid->output = (pid->proportional) + (KI * pid->integral) - (KD * pid->derivative);

	// Constrain angle PID output to PWM range
	pid->output = pid->direction * constrain(pid->output, -PID_LIMIT, PID_LIMIT);

	// Store variable for next iteration
	pid->last_input = pid->input;
	pid->last_error = pid->error;
	pid->last_time  = pid->time;
}

void PID_SetPoint(float throttle, float pitch, float roll, float yaw)
{
	pid_altitude.set_point = throttle;
	pid_pitch.set_point = pitch;
	pid_roll.set_point = roll;
	pid_yaw.set_point = yaw;
}

void PID_SetGains(int instance, float ckp, float cki, float ckd, float akp, float aki, float akd)
{
	switch (instance)
	{
		case PITCH:
			pid_pitch.con_KP = ckp;
			pid_pitch.con_KI = cki;
			pid_pitch.con_KD = ckd;
			pid_pitch.agr_KP = akp;
			pid_pitch.agr_KI = aki;
			pid_pitch.agr_KD = akd;
			break;

		case ROLL:
			pid_roll.con_KP = ckp;
			pid_roll.con_KI = cki;
			pid_roll.con_KD = ckd;
			pid_roll.agr_KP = akp;
			pid_roll.agr_KI = aki;
			pid_roll.agr_KD = akd;
			break;

		case YAW:
			pid_yaw.con_KP = ckp;
			pid_yaw.con_KI = cki;
			pid_yaw.con_KD = ckd;
			pid_yaw.agr_KP = akp;
			pid_yaw.agr_KI = aki;
			pid_yaw.agr_KD = akd;
			break;

		case ALT:
			pid_altitude.con_KP = ckp;
			pid_altitude.con_KI = cki;
			pid_altitude.con_KD = ckd;
			pid_altitude.agr_KP = akp;
			pid_altitude.agr_KI = aki;
			pid_altitude.agr_KD = akd;
			break;
	}
}

void PID_Init()
{
	/* Set Controller Direction */
	pid_pitch.direction = DIRECT;
	pid_roll.direction = REVERSE;
	pid_yaw.direction = DIRECT;

	/* Set PID Loop Time */
	pid_pitch.delta = 1;
	pid_roll.delta = 1;
	pid_yaw.delta = 1;

	/* Set Breakpoint Value */
	pid_pitch.breakpoint = 1500;
	pid_roll.breakpoint = 1450;
	pid_yaw.breakpoint = 2000;

	/* Set CG Offset Value */
	pid_pitch.offset = 3.5;
	pid_roll.offset = 0;
	pid_yaw.offset = 0;

	/* Set Gains */
	PID_SetGains(PITCH, 3.8f, 0.01f, 340.0f, 4.5f, 0.02f, 360.0f);
	PID_SetGains(ROLL, 3.2f, 0.01f, 360.0f, 3.8f, 0.02f, 320.0f);
	PID_SetGains(YAW, 3.0f, 0, 0, 0, 0, 0);
}

void PID_Update()
{
	pid_pitch.input = AHRS_GetPitch();
	pid_roll.input = AHRS_GetRoll();
	pid_yaw.input = AHRS_GetYaw();

#ifdef PID_DEBUG
	serialInt(pid_pitch.input);
	serialWrite('\t');
	serialInt(pid_roll.input);
	serialWrite('\t');
	serialInt(pid_yaw.input);
	serialWrite('\t');
#endif

	/* Emergency Power Down */
	if ((abs(pid_pitch.input) > 90) || (abs(pid_roll.input) > 90))
	{
		Motor_StopAll();
		toggleLED(0, 1, 0);
		return;
	}
	else toggleLED(1, 1, 1);

	PID_Compute(&pid_pitch);
	PID_Compute(&pid_roll);
	PID_Compute(&pid_yaw);

	/** TODO: Temporary, interface barometer */
	//pid_pitch.output = 0;
	//pid_roll.output = 0;
	//pid_yaw.output = 0;
	//pid_altitude.output = 1300;

	//Motor_DistributeSpeed(pid_altitude.output, pid_pitch.output, pid_roll.output, pid_yaw.output);
	//return;

	if (MOTOR_ARM) Motor_DistributeSpeed(pid_altitude.output, pid_pitch.output, pid_roll.output, pid_yaw.output);
	else Motor_StopAll();
}
