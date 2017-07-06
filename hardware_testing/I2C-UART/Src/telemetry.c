/*
 * telemetry.c
 *
 *  Created on: Jun 4, 2017
 *      Author: Heethesh
 */

#include "serial.h"
#include "telemetry.h"
#include "string.h"

struct txFrame txf;

void sendFrame()
{
	unsigned char checksum = 0;
	unsigned char txBuffer[12];

	printChar('<');
	memcpy(txBuffer, &txf, 12);

	for (int i=0; i<12; i++)
	{
		checksum += txBuffer[i];
		printChar(txBuffer[i]);
	}

	checksum = 0xFF - (0xFF & (unsigned char)checksum);

	printChar(checksum);
	printChar('>');
}
