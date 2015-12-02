/*
 * filter.h
 *
 *  Created on: 18.11.2015
 *      Author: aott
 */

#include "../stdint.h"
#include <math.h>

#include "../b_errorcodes.h"


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
int8_t filterSensorData(int16_t* avgSensorData, int16_t* filteredSensorData);


