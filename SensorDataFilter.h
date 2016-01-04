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

#ifndef SENSORDATAFILTER_H
#define SENSORDATAFILTER_H

#include <stdint.h>

#define ACC_X_IDX 0
#define ACC_Y_IDX 1
#define ACC_Z_IDX 2
#define GYR_X_IDX 3
#define GYR_Y_IDX 4
#define GYR_Z_IDX 5
#define MAG_X_IDX 6
#define MAG_Y_IDX 7
#define MAG_Z_IDX 8

#define EULER_ROLL_INDEX	0
#define EULER_PITCH_INDEX	1
#define EULER_YAW_INDEX		2

/**
 * This method filters previous averaged sensor data form the SensorDataManager
 *
 * @params
 * 	input: int16_t* avgSensorData
 * 			pointer to the averaged data.
 * 			This should have 9 values in the following order: angle_x/y/z, omega_/x/y/z, magnet_x/y/z
 * 			Where angle is accelerometer, omega is gyroscope and compass
 *
 * 	output: int16_t* filteredSensorData
 * 			pointer where the filtered data should be saved
 */
int8_t filterSensorData(int16_t* avgSensorData, float* filteredSensorData, uint32_t averagedDataDeltaT);
#endif
