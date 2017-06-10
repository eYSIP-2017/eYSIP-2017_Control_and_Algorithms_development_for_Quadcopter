/*
 * telemetry.c
 *
 *  Created on: Jun 4, 2017
 *      Author: Heethesh
 */

/* Library depreciated, use MSP */

#include "serial.h"
#include "telemetry.h"
#include <string.h>

struct txFrame txf;

void sendFrame()
{
	unsigned char checksum = 0;
	unsigned char txBuffer[12];

	// Start frame limiter
	serialWrite('<');

	// Convert struct elements to byte array
	memcpy(txBuffer, &txf, 12);

	// Transmit data payload
	for (int i=0; i<12; i++)
	{
		checksum += txBuffer[i];
		serialWrite(txBuffer[i]);
	}

	// Calculate checksum
	checksum = 0xFF - (0xFF & (unsigned char)checksum);
	serialWrite(checksum);

	// End frame limiter
	serialWrite('>');
}
