/*
 * multiwii.c
 *
 *  Created on: June 8, 2017
 *      Author: Heethesh
 */

#include "msp.h"
#include "stm32f1xx_hal.h"
#include "serial.h"
#include <string.h>

/* Transfer packets struct declarations */
msp_status msp_txf_status;
msp_raw_imu msp_txf_raw_imu;
msp_motor msp_txf_motor;
msp_rc msp_txf_rc;
msp_attitude msp_txf_attitude;
msp_altitude msp_txf_altitude;
msp_analog msp_txf_analog;
msp_rc_tuning msp_txf_rc_tuning;
msp_pid msp_txf_pid;
msp_misc msp_txf_misc;
msp_motor_pins msp_txf_motor_pins;
msp_boxnames msp_txf_boxnames;
msp_pidnames msp_txf_pidnames;
msp_boxids msp_txf_boxids;

/* Received packets struct declarations */
msp_set_raw_rc msp_rxf_raw_rc;
msp_set_pid msp_rxf_pid;
msp_set_box msp_rxf_box;
msp_set_rc_tuning msp_rxf_rc_tuning;
msp_set_misc msp_rxf_misc;
msp_set_head msp_rxf_head;
msp_set_motor msp_rxf_motor;

void MSP_SendFrame(uint8_t code, uint8_t *data, uint16_t data_length)
{
	uint8_t checksum = 0;

	// Send Header
	serialPrint("$M>");
	serialWrite(data_length);
	serialWrite(code);
	checksum = code ^ data_length;

	for (int i=0; i<data_length; i++)
	{
		serialWrite((char) data[i]);
		checksum ^= data[i];
	}
	serialWrite(checksum);
}

/** MSP RX under development */
uint8_t MSP_RecieveFrame(uint8_t code, uint8_t *data, uint16_t data_length)
{
	uint8_t checksum = 0;

	uint8_t temp_buff[data_length];
	serialReadBytes(&temp_buff, data_length);
	for (int i=0; i<data_length; i++)
		serialWrite((char) temp_buff[i]);

	/*if (serialRead() != '$') return 0;
	if (serialRead() != 'M') return 0;
	if (serialRead() != '<') return 0;
	if (serialRead() != (uint8_t) data_length) return 0;
	if (serialRead() != code) return 0;
	checksum = code^data_length;

	for (int i=0; i<data_length; i++)
	{
		temp_buff[i] = serialRead();
		checksum ^= temp_buff[i];
	}

	if (serialRead() != checksum) return 0;
	for (int i=0; i<data_length; i++)
		data[i] = temp_buff[i];
	return 1;*/
}

/**********************************
 Function name	:	MSP_SendStatus
 Functionality	:	To send status data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendStatus()
 ***********************************/
void MSP_SendStatus()
{
	uint16_t data_length = sizeof(msp_status);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_status, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_STATUS, &buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRawIMU
 Functionality	:	To send raw IMU data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRawIMU()
 ***********************************/
void MSP_SendRawIMU()
{
	uint16_t data_length = sizeof(msp_raw_imu);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_raw_imu, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_RAW_IMU, &buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMotor
 Functionality	:	To send motor data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMotor()
 ***********************************/
void MSP_SendMotor()
{
	uint16_t data_length = sizeof(msp_motor);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_motor, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_MOTOR, &buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRC
 Functionality	:	To send RC data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRC()
 ***********************************/
void MSP_SendRC()
{
	uint16_t data_length = sizeof(msp_rc);					// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_rc, data_length);					// Convert struct elements to bytes
	MSP_SendFrame(MSP_RC, &buff, data_length);				// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAttitude
 Functionality	:	To send attitude data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAttitude()
 ***********************************/
void MSP_SendAttitude()
{
	uint16_t data_length = sizeof(msp_attitude);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_attitude, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_ATTITUDE, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAltitude
 Functionality	:	To send altitude data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAltitude()
 ***********************************/
void MSP_SendAltitude()
{
	uint16_t data_length = sizeof(msp_altitude);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_altitude, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_ALTITUDE, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendAnalog
 Functionality	:	To send analog data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendAnalog()
 ***********************************/
void MSP_SendAnalog()
{
	uint16_t data_length = sizeof(msp_analog);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_analog, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_ANALOG, &buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendRCTuning
 Functionality	:	To send RC tuning data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendRCTuning()
 ***********************************/
void MSP_SendRCTuning()
{
	uint16_t data_length = sizeof(msp_rc_tuning);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_rc_tuning, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_RC_TUNING, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendPID
 Functionality	:	To send PID data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendPID()
 ***********************************/
void MSP_SendPID()
{
	uint16_t data_length = sizeof(msp_pid);					// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_pid, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_PID, &buff, data_length);				// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMisc
 Functionality	:	To send miscellaneous data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMisc()
 ***********************************/
void MSP_SendMisc()
{
	uint16_t data_length = sizeof(msp_misc);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_misc, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_MISC, &buff, data_length);			// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendMotorPins
 Functionality	:	To send motor pins data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendMotorPins()
 ***********************************/
void MSP_SendMotorPins()
{
	uint16_t data_length = sizeof(msp_motor_pins);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_motor_pins, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_MOTOR_PINS, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendBoxNames
 Functionality	:	To send box names data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendBoxNames()
 ***********************************/
void MSP_SendBoxNames()
{
	uint16_t data_length = sizeof(msp_boxnames);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_boxnames, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_BOXNAMES, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendPIDNames
 Functionality	:	To send box IDs data using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendPIDNames()
 ***********************************/
void MSP_SendPIDNames()
{
	uint16_t data_length = sizeof(msp_pidnames);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_pidnames, data_length);			// Convert struct elements to bytes
	MSP_SendFrame(MSP_PIDNAMES, &buff, data_length);		// Pack into MSP frame and transmit
}

/**********************************
 Function name	:	MSP_SendBoxIDs
 Functionality	:	To send SendBoxids using MSP
 Arguments		:	None
 Return Value	:	None
 Example Call	:	MSP_SendBoxIDs()
 ***********************************/
void MSP_SendBoxIDs()
{
	uint16_t data_length = sizeof(msp_boxids);				// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	memcpy(buff, &msp_txf_boxids, data_length);				// Convert struct elements to bytes
	MSP_SendFrame(MSP_BOXIDS, &buff, data_length);			// Pack into MSP frame and transmit
}

/** TODO: MSP RX under development */

uint8_t MSP_RecieveMotor()
{
	uint16_t data_length = sizeof(msp_set_motor);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	if(MSP_RecieveFrame(MSP_SET_MOTOR, &buff, data_length))	// Receive MSP frame and unpack data
	{
		memcpy(&msp_rxf_motor, buff, data_length);			// Convert bytes to struct elements
		return 1;											// Success
	}
	else return 0;											// Failure
}

uint8_t MSP_RecieveRC()
{
	uint16_t data_length = sizeof(msp_set_raw_rc);			// Get payload size
	uint8_t buff[data_length];								// Payload buffer
	if(MSP_RecieveFrame(MSP_SET_RAW_RC, &buff, data_length))// Receive MSP frame and unpack data
	{
		memcpy(&msp_rxf_raw_rc, buff, data_length);			// Convert bytes to struct elements
		return 1;											// Success
	}
	else return 0;											// Failure
}
