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
#include <math.h>
#include "PIDs/PIDPitch.h"
#include "PIDs/PIDRoll.h"
#include "PIDs/PIDYaw.h"
#include "PIDToMotorMapper.h"
#include "sys/alt_timestamp.h" //for time measurement
//#include "b_pwmdriver.h" // RC required for testing



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
	struct logData* rxData = malloc(sizeof(struct logData));
	while (1) {
		//reading latest byte out of LOGGER OS Message Que. 2nd argument is OSQPend() is timeout value
		//OSQPend() should block Task if no new data is in Queue
		rxData = OSQPend(loggerQsem, 0, &err); //returns 0
		if (err != OS_NO_ERR) {
			printf("Logging ERROR Code: %d\n", err);
		}

		//TODO send rxLoggerData to MCAPI
		//MCAPI format is defined in struct Logdata.

		//TODO Error handling on ERRORCODE 30 (Que Full) e.g. reset Que
		printf("LoggerTask running: avgData= %d\n", rxData->raw[0]);

	}
}


/*
 * Main Task
 */
void MainTask(void* pdata) {
//	int frameDone = 0; // RC required
	printf("Starting Main task...\n");

	int16_t avgSensorData[9] = { 0 }; //Order: ACC- GYR - COMP
	uint32_t averagedDataDeltaT = 0;
	float filteredSensorData[9] = { 0 }; //Order: ACC- GYR - COMP
	INT8U err = OS_ERR_NONE;

	struct logData* loggData = malloc(sizeof(struct logData));

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
//		if (SDM_NEW_DATA_AVAILABLE == 1) {
			err = getSensorData(avgSensorData, &averagedDataDeltaT);
			err = filterSensorData(avgSensorData, filteredSensorData, averagedDataDeltaT); //only filter new data


			//Logging / sensor debugging console-output
//			int i;
//			//printf("A:");
//			for (i = 0; i < 9; ++i) {
//				if (i % 3 == 0)
//					//printf("\t");
//
//				err = OSQPost(loggerQsem, (void*) avgSensorData[i]); //write new Logging Data in MSG Que
//				//			err = OSQPost(loggerQsem, (void*) filteredSensorData[i]);
//
//				//printf("%d \t", (int16_t) avgSensorData[i]);
//				if (err != OS_ERR_NONE) {
//					printf("ERROR occured Code: %d\n", err);
//					err = 0;
//				}
//			}
//			//printf("\n");
//			printf("F:");
//			for (i = 0; i < 9; ++i) {
//				if (i % 3 == 0)
//					printf("\t");
//
//				printf("%.3f\t", (float) filteredSensorData[i] * (180.0F / M_PI));
//				if (err != OS_ERR_NONE) {
//					printf("ERROR occured Code: %d\n", err);
//					err = 0;
//				}
//			}
//			printf("\n");


//			RC_SCALE_TO_PWM = rcValue to PWM output interval [0;210]
//			 max RCvalue [6400] * RC_SCALE_TO_PWM = PWM value

			//PID interval [-14; 14]
			float rcValueScale = 0.05625; //360/6400 - 180;

			//assuming X axis from Filter is PITCH
			float pidPITCHMidVal = PIDPitchCalculation(0, (float) filteredSensorData[0]); //PID value at medium rcInput = [472; 473] // Pitch Axis angle, mid RCvalue is 3200 * RC_SCALE_TO_PWM

			//assuming X axis from Filter is ROLL
			float pidROLLMidVal = PIDRollCalculation(0, (float) filteredSensorData[1]); //PID value at medium rcInput = [472; 473] // Pitch Axis angle, mid RCvalue is 3200 * RC_SCALE_TO_PWM

			//assuming X axis from Filter is YAW
			float pidYAWMidVal = PIDYawCalculation(0, (float) filteredSensorData[2]); //PID value at medium rcInput = [472; 473] // Pitch Axis angle, mid RCvalue is 3200 * RC_SCALE_TO_PWM

			printf("PITCH: %f\tROLL: %f\tYAW: %f\n", pidPITCHMidVal, pidROLLMidVal, pidYAWMidVal);

			mapToMotors(60, pidROLLMidVal, pidPITCHMidVal, pidYAWMidVal);

			//TODO PID to motor Mapping. The PID values [-14; 14] have to be scaled properly as well
//		}

		//evaluate standard pid error values to calculate offsets and scale factors




		//PID construct, differ between basic balancing and restoring:
		/**
		 * (rcCommand, ACC) -> (Balance PID err AND GYRO) -> MOTOR output
		 */
		//float pidPitchACC = PIDPitchCalculation(rcValue[2] * 0.0328125f, filteredSensorData[ACC_Y_IDX]); //pitch should get Pitch related values of Sensors
		//Restore mode hold horizon
		/*
		float pidPitchGYR = PIDPitchCalculation(pidPitchACC,
				filteredSensorData[GYR_X_IDX]); //2nd Stage PID
		*/

//		float pidRollACC = PIDRollCalculation(rcValue[3],
//				filteredSensorData[ACC_X_IDX]); //pitch should get Pitch related values of Sensors
//
//		/*
//		//Restore mode hold horizon
//		float pidRollGYR = PIDPitchCalculation(pidRollACC,
//				filteredSensorData[GYR_Y_IDX]); //2nd Stage PID
//		*/
//
//		float pidYaw = PIDYawCalculation(rcValue[0],
//				filteredSensorData[ACC_Z_IDX]); //Yaw getting Compasvalues
//
//		mapToMotors(rcValue[1], pidPitchGYR, pidPitchGYR, pidYaw); //map pid values to Motor ouput



		//Copy new Data to Struct, to send
		int i;
		for (i = 0; i < 9; ++i) { //copy logging data to txStruct
			loggData->raw[i] = avgSensorData[i];
			loggData->filter[i] = filteredSensorData[i];
		}

		loggData->pid[0] = pidPITCHMidVal;
		loggData->pid[1] = pidROLLMidVal;
		loggData->pid[2] = pidYAWMidVal;

		int j;
		for (j = 0; j < 8; ++j) {
			loggData->rawRadio[i] = rcValue[i];
		}
//		OSQPost(loggerQsem, (void*) loggData);
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
	rcReceiverMutex = OSMutexCreate(RC_RECEIVER_MUTEX_PRIORITY, &err);

	mainTaskSem = OSSemCreate(0);
	sensorDataManageTaskrSem = OSSemCreate(0);
	rcTaskSem = OSMutexCreate(SENSOR_DATA_MUTEX_PRIORITY, &err);

	alt_alarm_start(&periodicMainTaskAlarm, alt_ticks_per_second() * 2,
			mainTasktimerCallback, NULL); // periodic timer for MainTask

	alt_alarm_start(&periodicRCReceiverTaskAlarm, alt_ticks_per_second() * 2,
			RCReceiverTaskTasktimerCallback, NULL); // periodic timer for MainTask

	alt_alarm_start(&periodicSensorDataManagerTasktimerAlarm,
			alt_ticks_per_second() * 2, SensorDataManagerTasktimerCallback,
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
