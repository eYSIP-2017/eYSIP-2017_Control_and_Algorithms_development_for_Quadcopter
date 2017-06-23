/*
 * circular_buffer.c
 *
 *  Created on: June 13, 2017
 *      Author: Heethesh
 */

#include <stdint.h>
#include "circular_buffer.h"

CircularBuffer rxc;

int CB_Write(CircularBuffer *cb, uint8_t data)
{
	if ((cb->size >= CBUF_SIZE) && !CBUF_OVERWRITE) return 0;

	cb->size++;
	cb->buffer[cb->tail] = data;
	cb->tail = (cb->tail+1) % CBUF_SIZE;
	return 1;
}

int CB_Read(CircularBuffer *cb, uint8_t *data)
{
	if (cb->size == 0) return 0;

	cb->size--;
	*data = cb->buffer[cb->head];
	cb->head = (cb->head+1) % CBUF_SIZE;
	return 1;
}

uint16_t CB_Size(CircularBuffer *cb)
{
	return (cb->size);
}

void CB_Init(CircularBuffer *cb)
{
	cb->head = 0;
	cb->tail = 0;
	cb->size = 0;
	for (int i=0; i<CBUF_SIZE; i++)
		cb->buffer[i] = 0;
}
