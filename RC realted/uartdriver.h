/*
 * b_uartdriver.h
 *
 * Providing Functions to control the RS232-UART IP core
 *
 * find register information in:
 * documentation/n2cpu_nii51010.pdf
 *
 *  Created on: 28.10.2015
 *      Author: hoeft
 */

#ifndef B_UARTDRIVER_H_
#define B_UARTDRIVER_H_

//-----------------------Includes----------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include "includes.h"

extern INT8U* err;
extern OS_EVENT* uartQsem;

/**
 * Function init initializes the UART IP-Core.
 *
 * @param baud = desired Baudrate value
 */
void diverInitUART(int baud);

/**
 * resteting message que
 */
void flushUARTBuffer();

/**
 * Reading latest received Byte
 */
unsigned char serialRead(INT16U timeouts);


#endif /* B_UARTDRIVER_H_ */
