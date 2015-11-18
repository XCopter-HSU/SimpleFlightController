/*
 * PID_Yaw.h
 *
 *  Created on: 28.10.2015
 *      Author: aott
 */

#ifndef DRIVER_UART_H_
#define DRIVER_UART_H_

//-----------------------Includes-------------
#include <stdio.h>
#include <stdint.h>
#include "includes.h"

//extern INT8U* err;
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
 * waiting for timeout for Byte if desired
 */
unsigned char serialRead(INT16U timeout);


#endif /* DRIVER_UART_H_ */
