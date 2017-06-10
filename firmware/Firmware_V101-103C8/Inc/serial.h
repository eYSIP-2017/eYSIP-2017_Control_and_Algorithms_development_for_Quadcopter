/*
 * uart.h
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void serialWrite(unsigned char chr);
void serialWriteBytes(unsigned char *data, unsigned int len);
void serialPrint(char* str);
void serialLine(void);
void serialInt(int val);
void serialFloat(float val);

/* Under development */
unsigned char serialRead();
unsigned char serialReadBytes(unsigned char* data, unsigned int len);

#endif /* SERIAL_H_ */
