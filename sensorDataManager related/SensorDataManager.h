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


#include "b_errorcodes.h"
#include "stdint.h" // Include stdint.h for the use of Integers with a defined size


/**
 * read the Raw Sensorvalues directly from the sensors vial i2c
 */
int8_t readSensorData(int16_t* rawSensorData);

/**
 * read the averaged data from Task
 */
int8_t getSersorData(int16_t* avgSensorData);


/**
 * Call this Mehtode once in the SensorDataManagerTask.
 */
void SensorDataManager();



// init all Sensors
int8_t initSensors();
