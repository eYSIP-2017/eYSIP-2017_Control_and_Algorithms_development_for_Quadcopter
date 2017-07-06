/*
 * peripherals.h
 *
 *  Created on: May 31, 2017
 *      Author: Heethesh
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

void Peripherals_Init(void);
void _Error_Handler(char *, int);

#endif /* PERIPHERALS_H_ */
