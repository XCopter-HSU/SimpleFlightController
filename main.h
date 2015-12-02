/*
 * main.h
 *
 *  Created on: 05.11.2015
 *      Author: hoeft
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

extern OS_EVENT* uartQsem; //message que for uart // RC required

extern OS_EVENT* loggerQsem; // logger required

extern OS_EVENT* sendorDataMutex;

extern OS_EVENT* rcReceiverMutex;

extern uint16_t rcValue[8]; //rc values will be updated by the RCcontroller

extern INT8U* err;

//extern void SensorDataManagerTask(); // external SensorTask


#endif /* MAIN_H_ */
