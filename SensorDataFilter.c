#include <math.h>
#include <stdint.h>
#include <stdlib.h>


#include "SensorDataFilter.h"
#include "Errorcodes.h"

#define true 1
#define false 0


 /*
 * accelerometer calibration constants (measured by hand)
 */
const float ax_offset = 13;      // calibration of accelerometer values:
const float ay_offset = -2; 		// (accelX*accelX) + (accelY*accelY) * (accelZ*accelZ) should be independant of the orientation of the accelerometer
const float az_offset = 28;

//accl range [-maxAbsScaleDelta, maxAbsScaleDelta]
#define maxAbsScaleDelta 300.0f
float ax_scale = maxAbsScaleDelta / 262.0f; //1.145038
float ay_scale = maxAbsScaleDelta / 267.0f;
float az_scale = maxAbsScaleDelta / 250.0f;

/*
 * unit conversion constants
 */
const float gyroSens = 14.375;           // convert lsb to rad/sec (gyro output)
const float dtor = M_PI / 180.0F;          // convert degree to rad
const float rtod = 180.0F / M_PI;          // convert rad to degree

/*
 * raw sensor data array
 */
int raw_values_acc[9]; // stores avg. raw sensor data: angle_x/y/z, omega_/x/y/z, magnet_x/y/z

/*
 * flags used in computation
 */
uint8_t first = true;
uint8_t reduced[3] = { false, false, false };

/*
 * measured value
 */
//measured position
//position x = angle
float x_m[3] = { 0.0, 0.0, 0.0 };
//measured velocity
//velocity v = angle velocity
float v_m[3] = { 0.0, 0.0, 0.0 };
//? velocity offsets?!
float vo_m[3] = { 0.0, 0.0, 0.0 };

/*
 * predicted values
 */
//position x
float x_p[3] = { 0.0, 0.0, 0.0 };
//velocity v
float v_p[3] = { 0.0, 0.0, 0.0 };

/*
 * deltas
 */
//position x = winkel
float x_d[3] = { 0.0, 0.0, 0.0 };
//velocity v = winkel geschw
float v_d[3] = { 0.0, 0.0, 0.0 };

/*
 *
 */
//position x
float x_s[3] = { 0.0, 0.0, 0.0 };
//velocity v
float v_s[3] = { 0.0, 0.0, 0.0 };

/*
 * alpha beta , mid gyro & accl factors
 */
float r_f[3] = { 0.3, 0.3, 0.3 };
float r_a[3] = { 0.4, 0.4, 0.4 };
float r_b[3] = { 0.1, 0.1, 0.1 };

/*
 * sensor data after norming
 */
//accelerometer
float angle[3] = { 0.0, 0.0, 0.0 };
//gyroscope
float omega[3] = { 0.0, 0.0, 0.0 };
//compass
float mag[3] = { 0.0, 0.0, 0.0 };

/*
 * final predicted values
 */
//accelerometer predicted
float angle_p[3] = { 0.0, 0.0, 0.0 };
//gyroscope predicted
float omega_p[3] = { 0.0, 0.0, 0.0 };

/*
 * get / compute the raw data as angle(accl, comp) or rad/s (gyro)
 *
 * output parameter
 *	- angle
 *		pointer on accelerometer angle array
 *	- omega
 *		pointer on predicted gyroscope angular velocity array
 *	- mag
 *		pointer on predicted compass angle array
 *
 * input parameter
 *	- raw_values
 *		pointer on the raw sensor values array
 */
void getRawData(float* angle, float* omega, float* mag, int16_t* raw_values, float deltaT) {

	// norm accelerometer
	float accelX = ((float) (raw_values[ACC_Y_IDX] - ay_offset)) * ay_scale; // calibration of x-accelerometer
	float accelY = ((float) (-raw_values[ACC_X_IDX] - ax_offset)) * ax_scale; // calibration of y-accelerometer
	float accelZ = ((float) (-raw_values[ACC_Z_IDX] - az_offset)) * az_scale; // calibration of z-accelerometer
	float g = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ); // the measured value of g should be independant of the sensors orientation

	// norm accelerometer
	accelX = accelX / g;
	accelY = accelY / g;
	accelZ = accelZ / g;

	//reference coordinate system translation (sensor specific)
	float omegaX = -raw_values[GYR_X_IDX] * dtor / gyroSens; // [rad/s]
	float omegaY = raw_values[GYR_Y_IDX] * dtor / gyroSens; // [rad/s]
	float omegaZ = raw_values[GYR_Z_IDX] * dtor / gyroSens; // [rad/s]

	//
	float magnetX = raw_values[MAG_X_IDX];
	float magnetY = raw_values[MAG_Y_IDX];
	float magnetZ = raw_values[MAG_Z_IDX];

	//generate accelerometer angles
	angle[0] = atan2(accelY, accelZ); // Roll angle around x axis, [-180, 180] deg, numerically unstable
	angle[1] = atan2(-accelX,
			((accelY * sin(angle[0])) + (accelZ * cos(angle[0]))));	// Pitch angle around y axis, restricted to [-90, 90] deg

	//magnetometer zur ausgleichung weil xcopter nicht tangential zur erde steht
	angle[2] = atan2((magnetZ * sin(angle[0]) - magnetY * cos(angle[0])),
			(magnetX * cos(angle[1]) + (magnetY * sin(angle[1]) * sin(angle[0]))
					+ (magnetZ * sin(angle[1]) * cos(angle[0])))); // Yaw angle around z axis. [-180, 180] deg

	//gyroscope
	omega[0] = omegaX;
	omega[1] = omegaY;
	omega[2] = omegaZ;

	//compass
	mag[0] = magnetX;
	mag[1] = magnetY;
	mag[2] = magnetZ;
}

/*
 * filters the raw data
 *
 * output parameter
 *	- angle_p
 *		pointer on predicted euler angle array
 *	- omega_p
 *		pointer on predicted angular velocity array
 *
 * input parameter
 *	- angle_m
 *		pointer on measured accelerometer angle array
 *	- omega_m
 *		pointer on measured gyroscope angular velocity array
 *
 *	- step
 *		the time delta between each iteration ?
 *		time delta for integration
 */
void getFilteredData(float* angle_p, float* omega_p, float* angle_m, float* omega_m, float deltaT) {

	//init
	if (first) {
		int i;

		for (i = 0; i < 3; i++) {
			x_m[i] = angle_m[i]; 		//warum x_m = x_p
			x_p[i] = angle_m[i];

			v_p[i] = omega_m[i];

			angle_p[i] = x_p[i];
			omega_p[i] = v_p[i];

			first = false;
		}


	}else {
		int i;
		for (i = 0; i < 3; i++) {

			v_m[i] = omega_m[i] - vo_m[i]; //adjust measured omega values for sensor offset
			x_m[i] += (v_m[i] * deltaT); //integrate adjusted omega values to calculate angles

			//normierung
			if (x_m[i] < -(180.0 * dtor))
				x_m[i] += (360.0 * dtor); // -180 deg. <= x_m[i]*rtod <= +180 deg.
			if (x_m[i] > (180.0 * dtor))
				x_m[i] -= (360.0 * dtor);

			if (i == 1) {                                         // pitch angle
				if (x_m[i] > (90.0 * dtor)) { // transformation of "0 deg.->90 deg.->180 deg." to "0 deg.->90 deg.->0 deg."
					x_m[i] = (180.0 * dtor) - x_m[i];
				} else if (x_m[i] < -(90.0 * dtor)) { // transformation of "0 deg.->-90 deg.->-180 deg." to "0 deg.->-90 deg.->0 deg."
					x_m[i] = -(180.0 * dtor) - x_m[i];
				}
			}

			//anti drift...
			//mix gyro and accl
			float diff = angle_m[i] - x_m[i];
			if (diff >= (180.0 * dtor)) {
				x_m[i] = angle_m[i] + r_f[i] * ((360.0 * dtor) - diff);
			} else if (diff <= -(180.0 * dtor)) {
				x_m[i] = angle_m[i] + r_f[i] * (-(360.0 * dtor) - diff);
			} else {   // abs(diff) <   (180.0 * dtor)
				x_m[i] = (r_f[i] * angle_m[i]) + ((1 - r_f[i]) * x_m[i]);
			}

			//
			if (x_m[i] < -(180.0 * dtor))
				x_m[i] += (360.0 * dtor); // -180 deg. <= x_m[i]*rtod <= +180 deg.
			if (x_m[i] > (180.0 * dtor))
				x_m[i] -= (360.0 * dtor);
			if (i == 1) {                                         // pitch angle
				if (x_m[i] > (90.0 * dtor)) { // transformation of "0 deg.->90 deg.->180 deg." to "0 deg.->90 deg.->0 deg."
					x_m[i] = (180.0 * dtor) - x_m[i];
				} else if (x_m[i] < -(90.0 * dtor)) { // transformation of "0 deg.->-90 deg.->-180 deg." to "0 deg.->-90 deg.->0 deg."
					x_m[i] = -(180.0 * dtor) - x_m[i];
				}
			}

			//delta
			x_d[i] = x_m[i] - x_p[i];
			v_d[i] = v_m[i] - v_p[i];

			//
			if (x_d[i] < -(180.0 * dtor))
				x_d[i] += (360.0 * dtor); // correct for discont. +180 deg. <-> -180 deg.
			if (x_d[i] > (180.0 * dtor))
				x_d[i] -= (360.0 * dtor); // always use shortest distance

			//performance
			if ((abs(x_d[i]) <= 1.0 * dtor) && (reduced[i] == false)) {
				r_a[i] = r_a[i] / 2.0;
				r_b[i] = r_b[i] / 2.0;
				reduced[i] = true;
			} else if ((abs(x_d[i]) > 1.0 * dtor) && (reduced[i] == true)) {
				r_a[i] = r_a[i] * 2.0;
				r_b[i] = r_b[i] * 2.0;
				reduced[i] = false;
			}

			//error residuum
			x_s[i] = x_p[i] + (r_a[i] * x_d[i]);
			v_s[i] = v_p[i] + (r_b[i] * v_d[i]);

			//
			x_p[i] = x_s[i] + (deltaT * v_s[i]);

			//
			if (x_p[i] < -(180.0 * dtor))
				x_p[i] += (360.0 * dtor); // -180 deg. <= x_p[i]*rtod <= +180 deg.

			if (x_p[i] > (180.0 * dtor))
				x_p[i] -= (360.0 * dtor);

			if (i == 1) {                                         // pitch angle
				if (x_p[i] > (90.0 * dtor)) { // transformation of "0 deg.->90 deg.->180 deg." to "0 deg.->90 deg.->0 deg."
					x_p[i] = (180.0 * dtor) - x_p[i];
				} else if (x_p[i] < -(90.0 * dtor)) { // transformation of "0 deg.->-90 deg.->-180 deg." to "0 deg.->-90 deg.->0 deg."
					x_p[i] = -(180.0 * dtor) - x_p[i];
				}
			}

			//
			v_p[i] = v_s[i];

			//
			angle_p[i] = x_p[i];
			omega_p[i] = v_p[i];
		}
	}
}


int8_t filterSensorData(int16_t* avgSensorData, float* filteredSensorData, uint32_t averagedDataDelatT){

	float deltaT = averagedDataDelatT / 1000.0; //update deltaT for all functions and calculate from millisecond to second

	//convert raw data into angles etc
	getRawData(angle, omega, mag, avgSensorData, deltaT);

	//filter and merge sensor data
	getFilteredData(angle_p, omega_p, angle, omega, deltaT);


	//accl_x predicted
	filteredSensorData[0] = angle_p[0];
	//accl_y predicted
	filteredSensorData[1] = angle_p[1];
	//accl_z predicted
	filteredSensorData[2] = angle_p[2];


	//gyro_x predicted
	filteredSensorData[3] = omega_p[0];
	//gyro_y predicted
	filteredSensorData[4] = omega_p[1];
	//gyro_z predicted
	filteredSensorData[5] = omega_p[2];


	//compass_x
	filteredSensorData[6] = mag[0];
	//compass_y
	filteredSensorData[7] = mag[1];
	//compass_z
	filteredSensorData[8] = mag[2];


	return NO_ERR;
}
