/*************************************************************************
 * Copyright (c) 2004 Altera Corporation, San Jose, California, USA.      *
 * All rights reserved. All use of this software and documentation is     *
 * subject to the License Agreement located at the end of this file below.*
 **************************************************************************
 * Description:                                                           *
 * The following is a simple hello world program running MicroC/OS-II.The *
 * purpose of the design is to be a very simple application that just     *
 * demonstrates MicroC/OS-II running on NIOS II.The design doesn't account*
 * for issues such as checking system call return codes. etc.             *
 *                                                                        *
 * Requirements:                                                          *
 *   -Supported Example Hardware Platforms                                *
 *     Standard                                                           *
 *     Full Featured                                                      *
 *     Low Cost                                                           *
 *   -Supported Development Boards                                        *
 *     Nios II Development Board, Stratix II Edition                      *
 *     Nios Development Board, Stratix Professional Edition               *
 *     Nios Development Board, Stratix Edition                            *
 *     Nios Development Board, Cyclone Edition                            *
 *   -System Library Settings                                             *
 *     RTOS Type - MicroC/OS-II                                           *
 *     Periodic System Timer                                              *
 *   -Know Issues                                                         *
 *     If this design is run on the ISS, terminal output will take several*
 *     minutes per iteration.                                             *
 **************************************************************************/

#include <stdio.h>
#include "includes.h"

#include <stdint.h>
#include "main.h"//contains all semaphores OS_EVENTs for external files // RC required
#include "RCReceiver.h" // RC required
#include "logger.h"
#include "SensorDataManager.h"
//#include "b_pwmdriver.h" // RC required for testing

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK SensorDataManagerTask_stk[TASK_STACKSIZE];
OS_STK MainTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define SDM_TASK_PRIORITY      3
#define MAIN_TASK_PRIORITY     4
#define SENSOR_DATA_MUTEX_PRIORITY      2

/* Definition of UART OS_Queu*/
#define UART_Q_SIZE 16 // RC required
OS_EVENT* uartQsem; //message que for uart
void* uartQmessageTable[UART_Q_SIZE]; //uart queue table // RC required

/* Definition of Semaphores*/
OS_EVENT* rcTasksem; //message que for uart
OS_EVENT* sendorDataMutex;

/*global variables*/
//INT8U dummy;
//INT8U* err = &dummy;
uint16_t rcValue[8]; //contains rc commands will be updated by the receiver task // RC required

/*
 * Main Task
 */
void MainTask(void* pdata) {
//	int frameDone = 0; // RC required
	printf("Starting RC task...\n");

	int16_t* avgSensorData[9] = {0};
	int8_t err = NO_ERR;
	while (1) {
//		frameDone = updateChannelsRC();
//		if (frameDone == 1)
//		{
//		  OSSemPost(rcTasksem);
//		}
		err = getSensorData(avgSensorData);
		int i;
		for (i = 0; i < 9; ++i) {
			if (i % 3 == 0) {
				printf("\n");
			}
			printf("%d\t", (int16_t) avgSensorData[i]);
		}
		printf("\n\n");

		OSTimeDlyHMSM(0, 0, 0, 500);
	}
}


/*
 * Starting point for SimpleFlightController
 */
int main(void) {

	printf("Starting Program\n");
	INT8U err = OS_NO_ERR;
	uartQsem = OSQCreate((void*)&uartQmessageTable, UART_Q_SIZE); //create Message Que for UART

	sendorDataMutex = OSMutexCreate(SENSOR_DATA_MUTEX_PRIORITY, &err);

	rcTasksem = OSSemCreate(0);

	initRCreceiver();
	/*
	 * create SensorDataManagerTask
	 * declared in SensorDataManager.h
	 */
	err = OSTaskCreateExt(SensorDataManagerTask,
	NULL, (void *) &SensorDataManagerTask_stk[TASK_STACKSIZE - 1],
	SDM_TASK_PRIORITY,
	SDM_TASK_PRIORITY, SensorDataManagerTask_stk,
	TASK_STACKSIZE,
	NULL, 0);

	/*
	 * create MainTask
	 */
	err = OSTaskCreateExt(MainTask,
	NULL, (void *) &MainTask_stk[TASK_STACKSIZE - 1],
	MAIN_TASK_PRIORITY,
	MAIN_TASK_PRIORITY, MainTask_stk,
	TASK_STACKSIZE,
	NULL, 0);
	OSStart();
	return 0;

}

/******************************************************************************
 *                                                                             *
 * License Agreement                                                           *
 *                                                                             *
 * Copyright (c) 2004 Altera Corporation, San Jose, California, USA.           *
 * All rights reserved.                                                        *
 *                                                                             *
 * Permission is hereby granted, free of charge, to any person obtaining a     *
 * copy of this software and associated documentation files (the "Software"),  *
 * to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
 * and/or sell copies of the Software, and to permit persons to whom the       *
 * Software is furnished to do so, subject to the following conditions:        *
 *                                                                             *
 * The above copyright notice and this permission notice shall be included in  *
 * all copies or substantial portions of the Software.                         *
 *                                                                             *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
 * DEALINGS IN THE SOFTWARE.                                                   *
 *                                                                             *
 * This agreement shall be governed in all respects by the laws of the State   *
 * of California and by the laws of the United States of America.              *
 * Altera does not recommend, suggest or require that this reference design    *
 * file be used in conjunction or combination with any other product.          *
 ******************************************************************************/
