/*
 * main.h
 *
 *  Created on: 05.11.2015
 *      Author: hoeft
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

extern INT8U* err;

//extern void SensorDataManagerTask(); // external SensorTask


/* Definition of Task Stacks */
#define   TASK_STACKSIZE       2048
OS_STK RCReceiverTask_stk[TASK_STACKSIZE];
OS_STK SensorDataManagerTask_stk[TASK_STACKSIZE];
OS_STK MainTask_stk[TASK_STACKSIZE];
OS_STK LoggerTask_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define RC_TASK_PRIORITY 			7
#define SDM_TASK_PRIORITY      		4
#define MAIN_TASK_PRIORITY     		5
#define LOGGER_TASK_PRIORITY     	3 // logger task must have a high priority, it will only runn if new data was written in Logging Queue
#define SENSOR_DATA_MUTEX_PRIORITY	2
#define RC_RECEIVER_MUTEX_PRIORITY  6

/* Definition of UART OS_Que*/
#define UART_Q_SIZE 128 // RC required
OS_EVENT* uartQsem; //message que for uart
void* uartQmessageTable[UART_Q_SIZE]; //uart queue table // RC required

/* Definition of LOGGING OS_Que*/
#define LOGGER_Q_SIZE 128 // RC required
OS_EVENT* loggerQsem; //message que for uart
void* loggerQmessageTable[LOGGER_Q_SIZE]; //uart queue table // RC required

/* Definition of Semaphores*/
OS_EVENT* mainTaskSem;
OS_EVENT* rcTaskSem;
OS_EVENT* sensorDataManageTaskrSem;

OS_EVENT* sensorDataMutex; //global mutex for SensorDataManager
OS_EVENT* rcReceiverMutex; //global mutex to secure the rcValue Array

/*altera alarms*/
static alt_alarm periodicMainTaskAlarm;
static alt_alarm periodicRCReceiverTaskAlarm;
static alt_alarm periodicSensorDataManagerTasktimerAlarm;

/*TASK PERIODS in milli sec*/
#define MAIN_TASK_PERIOD				20
#define SENSORDATAMANAGER_TASK_PERIOD	100
#define RCRECEIVER_TASK_PERIOD			50

/* TASK start up delay in seconds*/
#define MAIN_TASK_DELAY					3
#define RC_TASK_DELAY					2
#define SENSORDATA_TASK_DELAY			2

/*Logger Data struct*/
struct logData{
	int16_t raw[9];
	int16_t filter[9];
	float pid[3];
	int16_t rawRadio[8];
}; //LogFiles


#endif /* MAIN_H_ */
