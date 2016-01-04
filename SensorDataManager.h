/*
 *	SensorDataManager.h
 *
 *  Created on: 04.11.2015
 *      Author: aott
 *
 *
 *	This Manager should provide a Method which reads the values from all Sensors and write it to an array
 *
 *	int readSensorData(double*)
 *
 *	@return int 	-An Errorcode specified in errorcodes.h
 *
 *	@param double* 	-A Pointer to the Array where all the sensordata should be stored.
 *
 *
 *
 */

#ifndef SENSORDATAMANAGER_H_
#define SENSORDATAMANAGER_H_

#include <stdint.h> // Include stdint.h for the use of Integers with a defined size
#include "includes.h"


#include "Errorcodes.h"

extern OS_EVENT* sensorDataMutex;

/**
 * 1 if new Sensordata is available
 * 0 if no new sensordata is available
 */

extern int8_t SDM_NEW_DATA_AVAILABLE;

/**
 * read the Raw Sensorvalues directly from the sensors vial i2c
 */
int8_t readSensorData(int16_t* rawSensorData);

/**
 * returns the averaged data from Task
 * interface to use the SensorDataManager
 */
INT8U getSensorData(int16_t* avgSensorData, uint32_t* deltaTime);

int8_t initSensors();


/*
 * Method to compute the offset thats neccessary to calibrate the gyro.
 * The actual offsets will be saved in a local array, offsets are applied in the SensorDataManagerTask
 */
int8_t getGyroCalibrationOffset();


/**
 * Call this Mehtode once in the SensorDataManagerTask.
 */
extern void SensorDataManagerTask(void* pdata);


/**
 * This funktion does all the averaging work on the lokal arrays.
 * sould it be listet here in .h - file?
 *
 * int8_t avgAllArrays();
 */

#endif /* SENSORDATAMANAGER_H_*/
