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
#include "main.h"//contains all semaphores OS_EVENTs for external files // RC required#include "RCReceiver.h" // RC required#include "logger.h"
#include "SensorDataManager.h"
#include "SensorDataFilter.h"
//#include "b_pwmdriver.h" // RC required for testing

/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK RCReceiverTask_stk[TASK_STACKSIZE];
OS_STK SensorDataManagerTask_stk[TASK_STACKSIZE];
OS_STK MainTask_stk[TASK_STACKSIZE];
OS_STK LoggerTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define RC_TASK_PRIORITY 			6
#define SDM_TASK_PRIORITY      		3
#define MAIN_TASK_PRIORITY     		4
#define LOGGER_TASK_PRIORITY     	7
#define SENSOR_DATA_MUTEX_PRIORITY	2
#define RC_RECEIVER_MUTEX_PRIORITY  5

/* Definition of UART OS_Que*/
#define UART_Q_SIZE 128 // RC requiredOS_EVENT* uartQsem; //message que for uart
void* uartQmessageTable[UART_Q_SIZE]; //uart queue table // RC required

/* Definition of LOGGING OS_Que*/
#define LOGGER_Q_SIZE 16 // RC requiredOS_EVENT* loggerQsem; //message que for uart
void* loggerQmessageTable[LOGGER_Q_SIZE]; //uart queue table // RC required

/* Definition of Semaphores*/
OS_EVENT* mainTaskSem;
OS_EVENT* rcTaskSem;
OS_EVENT* sensorDataManageTaskrSem;

OS_EVENT* sensorDataMutex; //global mutex for SensorDataManager
OS_EVENT* rcReceiverMutex; //global mutex to secure the rcValue Array

/*global variables*/

static alt_alarm periodicMainTaskAlarm;
static alt_alarm periodicRCReceiverTaskAlarm;
static alt_alarm periodicSensorDataManagerTasktimerAlarm;

uint16_t rcValue[8]; //contains rc commands will be updated by the receiver task // RC required

/* Reading UART and parse a SUMD frame then opens a semaphore for other task */
void RCReceiverTask(void* pdata) {
	int frameDone = 0; // RC required
	printf("Starting RC task...\n");
	INT8U err = OS_NO_ERR;
	while (1) {
		//printf for timer testing
		OSSemPend(rcTaskSem, 0, &err);
//		printf("RC Task running\n");
//		frameDone = updateChannelsRC(); // RC required
//		if (frameDone == 1) // RC required
//		{
//		  OSSemPost(mainTaskSem);
//		}
	}
}

/*
 * Task which is sending logging Data
 */
void LoggerTask(void* pdata) {

	printf("Starting Logger task...\n");
	INT8U err = OS_NO_ERR;
	//int16_t rxLoggerData[9] = {0};
	int dataCounter = 0;
	while (1) {
		//reading latest byte out of LOGGER OS Message Que. 2nd argument is OSQPend() is timeout value
		//OSQPend() should block Task if no new data is in Queue
		int16_t rxLoggerData = (int16_t) OSQPend(loggerQsem, 0, &err); //returns 0
		if (err != OS_NO_ERR) {
			printf("Logging ERROR Code: %d\n", err);
		}

		//TODO send rxLoggerData to MCAPI
//		dataCounter++;
//		if(rxLoggerData != NULL){ //data is not new
//			if(dataCounter >= 18)
//			{
//				dataCounter = 0;
//				continue;
//			}
//
//			if(dataCounter < 8)
//			{
//				printf("LoggerTask running: avgData= %d\n", (int16_t) rxLoggerData);
//			}
//
//			if(dataCounter > 8)
//			{
//				printf("LoggerTask running: filteredData= %d\n", (int16_t) rxLoggerData);
//			}
//		}

	}
}

/**
 * function to post data to Logger Queue
 */
void postDataToQ(int16_t* data) {
	int i;
	for (i = 0; i < 9; i++) {
		OSQPost(loggerQsem, (void*) data[i]); //write new Logging Data in MSG Que
	}
}

/*
 * Main Task
 */
void MainTask(void* pdata) {
//	int frameDone = 0; // RC required
	printf("Starting Main task...\n");

	int16_t avgSensorData[9] = { 0 }; //Oder: ACC- GYR - COMP
	float filteredSensorData[9] = { 0 }; //Oder: ACC- GYR - COMP
	INT8U err = OS_ERR_NONE;
	while (1) {
		OSSemPend(mainTaskSem, 0, &err);
//		printf("Main Task running\n");
		//RC receiver Test
//		frameDone = updateChannelsRC();
//		if (frameDone == 1)
//		{
//		  OSSemPost(rcTasksem);
//		}

		//Sensor Test
		if (SDM_NEW_DATA_AVAILABLE == 1) {
			err = getSensorData(avgSensorData);
			err = filterSensorData(avgSensorData, filteredSensorData); //only filter new data

			//Logging
			int i;
			printf("A:");
			for (i = 0; i < 9; ++i) {
				if (i % 3 == 0)
					printf("\t");

				err = OSQPost(loggerQsem, (void*) avgSensorData[i]); //write new Logging Data in MSG Que
				//			err = OSQPost(loggerQsem, (void*) filteredSensorData[i]);

				printf("%d \t", (int16_t) avgSensorData[i]);
				if (err != OS_ERR_NONE) {
					printf("ERROR occured Code: %d\n", err);
					err = 0;
				}
			}
			printf("\nF:");
			for (i = 0; i < 9; ++i) {
				if (i % 3 == 0)
					printf("\t");

				printf("%.3f\t", (float) filteredSensorData[i]);
				if (err != OS_ERR_NONE) {
					printf("ERROR occured Code: %d\n", err);
					err = 0;
				}
			}
			printf("\n\n");

		}

		//PID construct, differ between basic balancing and restoring:
		/**
		 * (rcCommand, ACC) -> (Balance PID err AND GYRO) -> MOTOR output
		 */
		float pidPitchACC = PIDPitchCalculation(rcValue[RC_Pitch],
				filteredSensorData[ACC_X_IDX]); //pitch should get Pitch related values of Sensors
		//Restore mode hold horizon
		float pidPitchGYR = PIDPitchCalculation(pidPitchACC,
				filteredSensorData[GYR_X_IDX]); //2nd Stage PID

		float pidRollACC = PIDPitchCalculation(rcValue[RC_ROLL],
				filteredSensorData[ACC_Y_IDX]); //pitch should get Pitch related values of Sensors
		//Restore mode hold horizon
		float pidRollGYR = PIDPitchCalculation(pidRollACC,
				filteredSensorData[GYR_Y_IDX]); //2nd Stage PID

		float pidYaw = PIDYawCalculation(rcValue[RC_YAW],
				filteredSensorData[MAG_X_IDX]); //Yaw getting Compasvalues

		mapToMotors(rcValue[RC_THROTTLE], pidPitchGYR, pidPitchGYR, pidYaw); //map pid values to Motor ouput

		//periodic timer Test

//		postDataToQ(avgSensorData);
	}
}

/*
 * RCReceiver periodic timer Callback
 */
alt_u32 mainTasktimerCallback(void* context) {
	OSSemPost(mainTaskSem);
	return alt_ticks_per_second() * 200 / 1000; //must return the periode ticks
}

/*
 * SensorDataManager periodic timer Callback
 */
alt_u32 RCReceiverTaskTasktimerCallback(void* context) {
	OSSemPost(rcTaskSem);
	return alt_ticks_per_second() * 500 / 1000; //must return the periode ticks
}

/*
 * mainTask periodic timer Callback
 */
alt_u32 SensorDataManagerTasktimerCallback(void* context) {
	OSSemPost(sensorDataManageTaskrSem);
	return alt_ticks_per_second() * 100 / 1000; //must return the periode ticks
}

/**
 * init method to init Driver related Stuff
 */
void DriverInit() {
	uartQsem = OSQCreate((void*) &uartQmessageTable, UART_Q_SIZE); //create Message Queue for UART driver
	initRCreceiver();
	initSensors();
}

/**
 * method to initialize OS related stuff e.g. setup of periodic timers
 *
 * returs an errorcode if occured
 */
int OSinit() {
	INT8U err = OS_NO_ERR; //error variable for init errors

	loggerQsem = OSQCreate((void*) &loggerQmessageTable, LOGGER_Q_SIZE); //create Message Queue for LOGGER

	sensorDataMutex = OSMutexCreate(SENSOR_DATA_MUTEX_PRIORITY, &err);
	rcReceiverMutex = OSMutexCreate(RC_RECEIVER_MUTEX_PRIORITY, &err);

	mainTaskSem = OSSemCreate(0);
	sensorDataManageTaskrSem = OSSemCreate(0);
	rcTaskSem = OSSemCreate(0);

//	alt_alarm_start(&periodicMainTaskAlarm, alt_ticks_per_second()*5, mainTasktimerCallback, NULL); // periodic timer for MainTask
//
//	alt_alarm_start(&periodicRCReceiverTaskAlarm, alt_ticks_per_second()*5, RCReceiverTaskTasktimerCallback, NULL); // periodic timer for MainTask
//
//	alt_alarm_start(&periodicSensorDataManagerTasktimerAlarm, alt_ticks_per_second()*5, SensorDataManagerTasktimerCallback, NULL); // periodic timer for MainTask

	return err;
}

/*
 * Starting point for SimpleFlightController
 */
int main(void) {

	printf("Starting Program\n");
	printf("initialize components...");
	DriverInit();
//	OSInit();

	INT8U err = OS_NO_ERR; //error variable for init errors

	loggerQsem = OSQCreate((void*) &loggerQmessageTable, LOGGER_Q_SIZE); //create Message Queue for LOGGER

	sensorDataMutex = OSMutexCreate(SENSOR_DATA_MUTEX_PRIORITY, &err);

	mainTaskSem = OSSemCreate(0);
	sensorDataManageTaskrSem = OSSemCreate(0);
	rcTaskSem = OSSemCreate(0);

	alt_alarm_start(&periodicMainTaskAlarm, alt_ticks_per_second() * 5,
			mainTasktimerCallback, NULL); // periodic timer for MainTask

	alt_alarm_start(&periodicRCReceiverTaskAlarm, alt_ticks_per_second() * 5,
			RCReceiverTaskTasktimerCallback, NULL); // periodic timer for MainTask

	alt_alarm_start(&periodicSensorDataManagerTasktimerAlarm,
			alt_ticks_per_second() * 5, SensorDataManagerTasktimerCallback,
			NULL); // periodic timer for MainTask

	printf("done\n");

//	INT8U err = OS_NO_ERR;
	/**
	 * create RCReceiver Task
	 */
	OSTaskCreateExt(RCReceiverTask,
	NULL, (void *) &RCReceiverTask_stk[TASK_STACKSIZE - 1],
	RC_TASK_PRIORITY,
	RC_TASK_PRIORITY, RCReceiverTask_stk,
	TASK_STACKSIZE,
	NULL, 0);

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
	 * create LoggerTask
	 */
	err = OSTaskCreateExt(LoggerTask,
	NULL, (void *) &LoggerTask_stk[TASK_STACKSIZE - 1],
	LOGGER_TASK_PRIORITY,
	LOGGER_TASK_PRIORITY, LoggerTask_stk,
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
