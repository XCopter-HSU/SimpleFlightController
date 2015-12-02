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


int8_t getSensorData(int16_t* avgSensorData){

	INT8U err = OS_NO_ERR;
	int i;

	OSMutexPend(sensorDataMutex, 0, &err);//Acquire Mutex for the avg Data
	for(i = 0;i <9 ;i++){
		avgSensorData[i] = avgData[i];
	}
	SDM_NEW_DATA_AVAILABLE = 0;
//	newDataAvailable = 0;
	err = OSMutexPost(sensorDataMutex);//release Semaphore for the avg Data

	return err;
}


void SensorDataManagerTask(void* pdata){

	int8_t cnt = 0;

	int16_t rawData[9];

	int8_t err = NO_ERR;



	while(1){

		err = readSensorData(rawData); //get newRawData

		acclX[cnt] = rawData[0];	//fill arrays with new raw data
		acclY[cnt] = rawData[1];
		acclZ[cnt] = rawData[2];
		gyroX[cnt] = rawData[3];
		gyroY[cnt] = rawData[4];
		gyroZ[cnt] = rawData[5];
		compX[cnt] = rawData[6];
		compY[cnt] = rawData[7];
		compZ[cnt] = rawData[8];

		cnt++;

		if(cnt>=20){
			err = avgAllArrays();

			SDM_NEW_DATA_AVAILABLE = 1; //new data is available

//			newDataAvailable = 1;
		}

		cnt = cnt % VALUE_NUM; //cnt goes from 0 to VALUE_NUM and then starts with 0 again

		OSTimeDlyHMSM(0, 0, 0, 10); //uncomment if sensordata reading is to fast for the i2c interface -> set delay to appropriate value
	}
}



int8_t initSensors(){

	int8_t err1 = Accelerometer_init();
	int8_t err2 = Compass_init();
	int8_t err3 = Gyroscope_init();

	if (err1) {
		return err1;
	}
	if (err2) {
		return err2;
	}
	if (err3) {
		return err3;
	}
	return NO_ERR; //dont know how to return multiple erroros
}
