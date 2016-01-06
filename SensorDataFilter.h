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
#include <math.h>

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

 /*
 * accelerometer calibration constants (measured by hand)
 */
const float ax_offset = 13;      // calibration of accelerometer values:
const float ay_offset = -2; 		// (accelX*accelX) + (accelY*accelY) * (accelZ*accelZ) should be independant of the orientation of the accelerometer
const float az_offset = 28;

const float maxAbsScaleDelta = 300.0;
const float ax_scale = maxAbsScaleDelta/262.0; //1.145038
const float ay_scale = maxAbsScaleDelta/267.0;
const float az_scale = maxAbsScaleDelta/250.0;

/*
 * unit conversion constants
 */
const float gyroSens = 14.375;           // convert lsb to rad/sec (gyro output)
const float dtor = M_PI / 180.0F;          // convert degree to rad
const float rtod = 180.0F / M_PI;          // convert rad to degree

//TODO
//0.2 = 200ms because maintask periode is 200ms
const float deltaT = 0.01; // time steps in units of seconds   TODO:!!!!!!!GET STEPS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//TODO

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
