/*
 * RCreceiver.c
 *
 *  Created on: 04.11.2015
 *      Author: hoeft
 */

#include "RCreceiver.h"
#include "Drivers/Driver_UART.h"
#include <stdint.h>
//#include "includes.h" //UCOS ii stuff not requiered if driver takes care of OS functions

typedef unsigned char uchar_t;

int8_t RC_RECEIVER_NEW_DATA_AVAILABLE = 0; //update flag if current Data was updated successfully

uint16_t rcValue[SUMD_MAXCHAN]; //extern value will be updated by updateChannelsRC, declared in main.h

static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t sumd[SUMD_BUFFSIZE]={0};


///**
// * > mfw driver would be too complicated, and has to be rewritten
// *
// * custom ISR will fill up raBytes
// *
// * will open semaphore if a SUMD-frame was fully received
// *
// * a custom ISR is would be complicated. there is a simpler way
// */
//void sumdISR() //must be defined above initReceiver
//{
//	static uint8_t sumdSize = 0;
//	if (sumdSize == SUMD_BUFFSIZE) {
//		sumdFrameFilled = 1;
//	}
////	sumd[sumdIndex++] = val;
//}


/**
 * init UART driver and register UART interrupts
 */
void initRCreceiver()
{
	diverInitUART(115200); //init uart
//	registerUARTinterruptISR(sumdISR);
//	registerUARTinterrupt(); // setup UART interrupt
}


uint8_t getRCvalues(uint16_t* newRCvalues)
{
	INT8U err = OS_NO_ERR;
	int i;

	OSMutexPend(rcReceiverMutex, 0, &err);//Acquire Mutex for the avg Data
	for(i = 0;i < SUMD_MAXCHAN;i++){
		newRCvalues[i] = rcValue[i];
	}
	RC_RECEIVER_NEW_DATA_AVAILABLE = 0;
//	newDataAvailable = 0;
	err = OSMutexPost(rcReceiverMutex);//release Semaphore for the avg Data

	return err;
}

/*
 * checks crc 16 of received raw frame data
 * @parameters
 * 	received raw frame data as unsigned char pointer
 * @return
 * 	1 	-> CRC good
 * 	-1 	-> CRC wrong
 */
uint8_t crcRawFrameData() {
	/* check crc before processing */
	/* SUMD has 16 bit CCITT CRC */
	uint16_t crc = 0;

	//crc only over header and data of frame
	int len = SUMD_BUFFSIZE - SUMD_CRC_LENGTH;
	int n=0, i=0;
	for (n = 0; n < len; n++) { //calculating crc sum for all bytes
		crc ^= (uint16_t) sumd[n] << 8;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}
	if (crc ^ (((uint16_t) sumd[len] << 8) | sumd[len + 1])) //last Byte is the crc Sum from the receiver
		/* wrong crc checksum found */
		return -1;
	else
		return 1;
}


/**
 * should be handle like a task
 */
int updateChannelsRC()
{
//	uchar_t rawByte = (uchar_t *)OSQPend(uartQsemMsg, 100, NULL);
	uchar_t rawByte = serialRead(0); //reading latest byte

	if (sumdIndex == 0 && rawByte != SUMD_SYNCBYTE)
	{
		flushUARTBuffer(); //empty que if it
		return 0; //replaced continue with return; //only start filling raw Bytes if it is the beginning of a new frame
	}
	if (sumdIndex == 2)
		sumdSize = rawByte; //the 3rd byte equals the frame size
	if (sumdIndex < SUMD_BUFFSIZE)
		sumd[sumdIndex] = rawByte; //fill raw bytes in array until the SUMD frame is completely read

	sumdIndex++;


	if (sumdIndex == sumdSize * 2 + 5) { //SUMD frame is now completely done, the actural receiver Channel data has to be evaluated now
		if(crcRawFrameData() != 1)
		{
			return 0; //exit if a crc Error occurred
		}

		sumdIndex = 0; //reset the index counter
		//debug[1] = sumd[1]; //the 2nd byte indicates if there is a connection failure to the Transmitter. has to be implemented if needed
		if (sumdSize > SUMD_MAXCHAN)
			sumdSize = SUMD_MAXCHAN; //prevent a out of range

		uint8_t *tmp = &(sumd[3]); //creating pointer to first high byte (3rd SUMD-frame)
		uint8_t b = 0;
		for (b = 0; b < sumdSize; b++)// appending high byte and the low byte to one value
		{
			rcValue[b] = (tmp[0] << 8) | tmp[1]; //removed >> 3 from MultiWii example
		//the RCvalues musst be converted to be compatible for

			rcValue[b] -= 8800; //now the range of the RC value goes from 0 to 6400

			//360/6400 = 0,05625 Value to scale rcValue to go from 0 to 360
			//add offset to make the rcValues go from -180 to 180 max

			//Pitch value should be between 90deg and -90deg
			//180/6400 = 0,028125 Scale RC to 90deg
			rcValue[b] *= (b == RC_PITCH_INDEX) ? 0.028125 : 0.05625;
			rcValue[b] -= (b == RC_PITCH_INDEX) ? 90 : 180;

			tmp += 2; //increasing pointer to next channel high byte
		}
		RC_RECEIVER_NEW_DATA_AVAILABLE = 1;
		return 1; //reading RX is done
	}
	return 0;
}
