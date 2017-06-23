/*
 * circular_buffer.h
 *
 *  Created on: Jun 13, 2017
 *      Author: Heethesh
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include "stm32f1xx_hal.h"

#define CBUF_SIZE 8192
#define CBUF_OVERWRITE 1

typedef struct
{
	volatile uint8_t  buffer[CBUF_SIZE]; 				// block of memory
	volatile uint16_t head; 						// holds current read position: 0 to (size-1)
	volatile uint16_t tail; 						// holds current write position: 0 to (size-1)
	volatile uint16_t size;
}CircularBuffer;

extern CircularBuffer rxc;

void CB_Init(CircularBuffer *cb);					// to initialize buffer
int CB_Write(CircularBuffer *cb, uint8_t data);		// writes in the buffer and increment head
int CB_Read(CircularBuffer *cb, uint8_t* data);		// read in the buffer and increment tail
uint16_t CB_Size(CircularBuffer *cb); 				// returns size of data present in buffer

#endif /* CIRCULAR_BUFFER_H_ */
