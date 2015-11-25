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
#include "Driver/Driver_Accl.h"
#include "Driver/Driver_Compa.h"
#include "Driver/Driver_Gyro.h"

#define VALUE_NUM 20

int16_t aX[VALUE_NUM] = {0};
int16_t aY[VALUE_NUM] = {0};
int16_t aZ[VALUE_NUM] = {0};
int16_t cX[VALUE_NUM] = {0};
int16_t cY[VALUE_NUM] = {0};
int16_t cZ[VALUE_NUM] = {0};
int16_t gX[VALUE_NUM] = {0};
int16_t gY[VALUE_NUM] = {0};
int16_t gZ[VALUE_NUM] = {0};
int16_t* arrs[9] = {aX, aY, aZ, cX, cY, cZ, gX, gY, gZ};

int16_t avgData[9] = {0};

int8_t readSensorData(int16_t* rawSensorData){

	int16_t acclX;
	getAccX(&acclX);

	int16_t acclY;
	getAccX(&acclY);

	int16_t acclZ;
	getAccX(&acclZ);


	int16_t compX;
	int16_t compY;
	int16_t compZ;
	Compass_getRawValues(&compX, &compY, &compZ);


	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	int16_t temp;

	getGyroAll(&temp, &gyroX, &gyroY, &gyroZ);


	rawSensorData[0] = acclX;
	rawSensorData[1] = acclY;
	rawSensorData[2] = acclZ;
	rawSensorData[3] = compX;
	rawSensorData[4] = compY;
	rawSensorData[5] = compZ;
	rawSensorData[6] = gyroX;
	rawSensorData[7] = gyroY;
	rawSensorData[8] = gyroZ;

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
	OSMutexPend(sendorDataMutex, 0, &err);
	for(i = 0; i<9 ;i++){
		avgData[i] = avgTempData[i];
	}
	//release Semaphore for the avg Data
	err = OSMutexPost(sendorDataMutex);

	return NO_ERR;
}


int8_t getSensorData(int16_t* avgSensorData){

	INT8U err = OS_NO_ERR;
	int i;

	OSMutexPend(sendorDataMutex, 0, &err);//Acquire Mutex for the avg Data
	for(i = 0;i <9 ;i++){
		avgSensorData[i] = avgData[i];
	}
	SDM_NEW_DATA_AVAILABLE = 0;
	err = OSMutexPost(sendorDataMutex);//release Semaphore for the avg Data

	return NO_ERR;
}


void SensorDataManagerTask(void* pdata){

	int8_t cnt = 0;

	int16_t rawData[9];

	int8_t err = NO_ERR;



	while(1){

		err = readSensorData(rawData); //get newRawData

		aX[cnt] = rawData[0];	//fill arrays with new raw data
		aY[cnt] = rawData[1];
		aZ[cnt] = rawData[2];
		cX[cnt] = rawData[3];
		cY[cnt] = rawData[4];
		cZ[cnt] = rawData[5];
		gX[cnt] = rawData[6];
		gY[cnt] = rawData[7];
		gZ[cnt] = rawData[8];

		cnt++;

		cnt = cnt%VALUE_NUM;

		err = avgAllArrays();

		SDM_NEW_DATA_AVAILABLE = 1; //new data is available

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
