/*
 * RCreceiver.c
 *
 *  Created on: 04.11.2015
 *      Author: hoeft
 */

#include "RCreceiver.h"
#include "uartdriver.h"
#include <stdint.h>
//#include "includes.h" //UCOS ii stuff not requiered if driver takes care of OS functions

typedef unsigned char uchar_t;

static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t sumd[SUMD_BUFFSIZE]={0};


///**
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
//	uchar_t* bytePointer = sumd[0];

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
		//debug[1] = sumd[1]; //the 2nd byte indicates if there is a connection failure to the Transmitter. has to be implemented first
		if (sumdSize > SUMD_MAXCHAN)
			sumdSize = SUMD_MAXCHAN; //prevent a out of range

		uint8_t *tmp = &(sumd[3]); //creating pointer to first high byte (3rd SUMD-frame)
		uint8_t b = 0;
		for (b = 0; b < sumdSize; b++)// appending high byte and the low byte to one value
		{
			rcValue[b] = (tmp[0] << 8) | tmp[1]; //removed >> 3
			tmp += 2; //increasing pointer to next channel high byte
		}
		return 1; //reading RX is done
	}
	return 0;
}

