/*
 * SensorDataManager.c
 *
 *  Created on: 28.10.2015
 *      Author: aott
 *
 *
 *
 */

#include "SensorDataManager.h"
#include "Drivers/Driver_Accl.h"
#include "Drivers/Driver_Compa.h"
#include "Drivers/Driver_Gyro.h"
#include <unistd.h>
#include <stdlib.h>

#define VALUE_NUM 20

int16_t acclX[VALUE_NUM] = {0};
int16_t acclY[VALUE_NUM] = {0};
int16_t acclZ[VALUE_NUM] = {0};

int16_t compX[VALUE_NUM] = {0};
int16_t compY[VALUE_NUM] = {0};
int16_t compZ[VALUE_NUM] = {0};

int16_t gyroX[VALUE_NUM] = {0};
int16_t gyroY[VALUE_NUM] = {0};
int16_t gyroZ[VALUE_NUM] = {0};

int16_t* arrs[9] = {acclX, acclY, acclZ, gyroX, gyroY, gyroZ, compX, compY, compZ};

//init all elements to 0
int16_t avgData[9] = {0};

int8_t SDM_NEW_DATA_AVAILABLE = 0;

int16_t gyroOffsets[3];

uint32_t averagedDataDeltaT = 0;

//int8_t newDataAvailable = 0;

int8_t readSensorData(int16_t* rawData){

	getAccX(&(rawData[0]));
	getAccY(&(rawData[1]));
	getAccZ(&(rawData[2]));

	int16_t temp; //drop it
	getGyroAll(&temp, &(rawData[3]), &(rawData[4]), &(rawData[5]));

	Compass_getRawValues(&(rawData[6]), &(rawData[7]), &(rawData[8]));

	return NO_ERR;
}

int8_t avgAllArrays(){
	INT8U err = OS_NO_ERR;


	int16_t avgTempData[9];

	int i,j;
	for (i = 0; i < 9; i++) {
		int16_t sum = 0;
		for (j = 0; j < VALUE_NUM; j++) {
			sum += (arrs[i])[j];
 		}
		avgTempData[i] = sum/VALUE_NUM;
	/**     // static average array with 5 values
	#define ARR_AVG5(arr) ((arr[0]+arr[1]+arr[2]+arr[3]+arr[4])/VALUE_NUM)
	avgTempData[0] = ARR_AVG5(aX);
	*/
	}

	//get Semaphore for the avg Data
	OSMutexPend(sensorDataMutex, 0, &err);
	for(i = 0; i<9 ;i++){
		avgData[i] = avgTempData[i];
	}
	//release Semaphore for the avg Data
	err = OSMutexPost(sensorDataMutex);

	return err;
}


int8_t getSensorData(int16_t* avgSensorData, uint32_t* deltaTime){

	INT8U err = OS_NO_ERR;
	int i;

	OSMutexPend(sensorDataMutex, 0, &err);//Acquire Mutex for the avg Data
	for(i = 0;i <9 ;i++){
		avgSensorData[i] = avgData[i];

	}
	*deltaTime = averagedDataDeltaT;
	SDM_NEW_DATA_AVAILABLE = 0;
	err = OSMutexPost(sensorDataMutex);//release Semaphore for the avg Data

	return err;
}


void SensorDataManagerTask(void* pdata){

	int8_t cnt = 0;

	int16_t rawData[9];

	int8_t err = NO_ERR;

	//start
	uint32_t start  = alt_nticks();
	uint32_t stop = 0;

	while(1){

		err = readSensorData(rawData); //get newRawData

		acclX[cnt] = rawData[0];	//fill arrays with new raw data
		acclY[cnt] = rawData[1];
		acclZ[cnt] = rawData[2];
		gyroX[cnt] = rawData[3] - gyroOffsets[0]; //sensor Offsets are only available for gyrodata
		gyroY[cnt] = rawData[4] - gyroOffsets[1];
		gyroZ[cnt] = rawData[5] - gyroOffsets[2];
		compX[cnt] = rawData[6];
		compY[cnt] = rawData[7];
		compZ[cnt] = rawData[8];

		cnt++;


		if(cnt>=20){

			err = avgAllArrays();

			stop =  alt_nticks();
			averagedDataDeltaT = (stop - start)*1000/alt_ticks_per_second(); //calculate the time needed to get all sensordata in milliseconds
			start = stop; //restart Timer

			SDM_NEW_DATA_AVAILABLE = 1; //new data is available
		}

		cnt = cnt % VALUE_NUM; //cnt goes from 0 to VALUE_NUM and then starts with 0 again

		OSTimeDlyHMSM(0, 0, 0, 10); //uncomment if sensordata reading is to fast for the i2c interface -> set delay to appropriate value
	}
}



int8_t initSensors(){

	int8_t err1 = Accelerometer_init();
	int8_t err2 = Compass_init();
	int8_t err3 = Gyroscope_init();
	err3 = getGyroCalibrationOffset();

	if (err1) {
		return err1;
	}
	if (err2) {
		return err2;
	}
	if (err3) {
		return err3;
	}
	return NO_ERR; //dont know how to return multiple errors
}


int8_t getGyroCalibrationOffset() {

	int8_t numberOfSamples = 100;
	int32_t dataThreshold = 5; // TODO: Determine good value!!!! 5 is only a guess

	int16_t rawGyroData[3];
	int32_t avgGyroData[3];
	int16_t gyroTmp = 0;


	int16_t j;

	int32_t difference[3] = {0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF}; // 0xFFFFFFF = 2147483647 = SingedInterger32.MAX_VALUE
	int32_t lastValues[3] = {0};



	while(difference[0] > dataThreshold ||
		  difference[1] > dataThreshold ||
		  difference[2] > dataThreshold){

		for(j = 0; j < numberOfSamples; j++) {
			//read gyro
			getGyroAll(&gyroTmp, &(rawGyroData[0]), &(rawGyroData[1]), &(rawGyroData[2]));

			//accumulate raw values
			avgGyroData[0] += rawGyroData[0];
			avgGyroData[1] += rawGyroData[1];
			avgGyroData[2] += rawGyroData[2];
			usleep(50); //measure gyrodata in different times, since the the sensor temperature changes
		}



		//divide accumulated raw values by numberOfSamples -> avarage!
		avgGyroData[0] /= numberOfSamples;
		avgGyroData[1] /= numberOfSamples;
		avgGyroData[2] /= numberOfSamples;

		//get avg difference from last averaged value and actual avaraged value (compare to treshold in while loop)
		difference[0] = abs(lastValues[0]-avgGyroData[0]);
		difference[1] = abs(lastValues[1]-avgGyroData[1]);
		difference[2] = abs(lastValues[2]-avgGyroData[2]);

		lastValues[0] = avgGyroData[0];
		lastValues[1] = avgGyroData[1];
		lastValues[2] = avgGyroData[2];

	}

	//saving final offsets
	gyroOffsets[0] = avgGyroData[0];
	gyroOffsets[1] = avgGyroData[1];
	gyroOffsets[2] = avgGyroData[2];

	return NO_ERR;
}
