#include "PIDToMotorMapper.h"
#include "Drivers/Driver_Motor.h"
#include "Errorcodes.h"
#include <stdio.h>

/*
Offsets and scale values depends on the defined ranges of the pid ranges

*/
float scaleFloat = 1000.0 ;
float scaleThrottleToPWM = (float)(210.0/360.0);
float scaleAxisToPWM = (float)(210.0/28000.0);
int offsetAxis = 14000;
int offsetThrottle = 180;
int pwmOffset = 8;

float scaleToPWM = 1.0;

/*PIDMIX calculates the value for each motor.
Each motor gets a roll, pitch and yaw value from the pids and also a throttle value from
the remote control. The full speed value of throttle is limited to a maximal value of 75%, so 
there are +- 25% reserved for roll and pitch. Roll pitch and yaw are multiplied with the 
x, y and z value of the mapping table. 
*/ 
float PIDMIX(uint8_t X, uint8_t Y, uint8_t Z, float _throttle, float _roll, float _pitch, float _yaw) {
	return (_throttle * 0.75) + (_roll * X) + (_pitch * Y) + (_yaw * Z);
}




//Maps all four values to each motor and computes the the PWM- signal for the motors.
int8_t mapToMotors(float throttle, float roll, float pitch, float yaw){
	
/*Maps the range of the pid values and the range of the throttle value into the range of the
PWM signal
*/	pidValues[0] = throttle * 28000.0/260.0 + offsetThrottle ;
	pidValues[1] = roll * scaleFloat + offsetAxis;
	pidValues[2] = pitch * scaleFloat + offsetAxis;
	pidValues[3] = yaw * scaleFloat + offsetAxis;

//	printf("Raw0: %f\tRaw1: %f\tRaw2: %f\tRaw3: %f\n\n", pidValues[0], pidValues[1], pidValues[2], pidValues[3]);

	#ifdef QUADX
		//Mapping table for a QUADX configuration
		motorQuadx[0] = PIDMIX(-1,+1,-1, pidValues[0],pidValues[1], pidValues[2], pidValues[3]); //REAR_R
		motorQuadx[1] = PIDMIX(-1,-1,+1, pidValues[0],pidValues[1], pidValues[2], pidValues[3]); //FRONT_R
		motorQuadx[2] = PIDMIX(+1,+1,+1, pidValues[0],pidValues[1], pidValues[2], pidValues[3]); //REAR_L
		motorQuadx[3] = PIDMIX(+1,-1,-1, pidValues[0],pidValues[1], pidValues[2], pidValues[3]); //FRONT_L
		
		printf("Mix0: %d\tMix1: %d\tMix2: %d\tMix3: %d\n\n", motorQuadx[0], motorQuadx[1], motorQuadx[2], motorQuadx[3]);
		
		motorQuadx[0] = (motorQuadx[0] + 7000) * 210/112000;
		motorQuadx[1] = (motorQuadx[1] + 7000) * 210/112000;
		motorQuadx[2] = (motorQuadx[2] + 7000) * 210/112000;
		motorQuadx[3] = (motorQuadx[3] + 7000) * 210/112000;


	#endif
	#ifdef HEX
//		motorHex[0] = PIDMIX(-1,+1,-1);															//	----TODO----
//		motorHex[1] = PIDMIX(-1,-1,+1);
//		motorHex[2] = PIDMIX(+1,+1,+1);
//		motorHex[3] = PIDMIX(+1,-1,-1);
//		motorHex[4] = PIDMIX(+1,-1,-1);
//		motorHex[5] = PIDMIX(+1,-1,-1);
	 #endif
	 
	writeToMotors();
	//TODO errors
	return NO_ERR;
 }
 

//Writes the PWM- signal to each motor. 
void writeToMotors()
{
	//Add an offset to the computed values because the PWM range is from 8 to 218
	motorQuadx[0] = motorQuadx[0] + pwmOffset;
	motorQuadx[1] = motorQuadx[1] + pwmOffset;
	motorQuadx[2] = motorQuadx[2] + pwmOffset;
	motorQuadx[3] = motorQuadx[3] + pwmOffset;
	
	printf("Mo0: %d\tMo1: %d\tMo2: %d\tMo3: %d\n\n", motorQuadx[0], motorQuadx[1], motorQuadx[2], motorQuadx[3]);

	int16_t err = NO_ERR;	
	#ifdef QUADX
		err = MotorDriver_setSpeed(motorQuadx[3], Motor_Front_Left);							
		if(err != NO_ERR)
		{
			//logerror(err)
		}

		err = MotorDriver_setSpeed(motorQuadx[1], Motor_Front_Right);
		if(err != NO_ERR)
		{
			//logerror(err)
		}

		err = MotorDriver_setSpeed(motorQuadx[2], Motor_Back_Left);
		if(err != NO_ERR)
		{
			//logerror(err)
		}

		err = MotorDriver_setSpeed(motorQuadx[0], Motor_Back_Right);
		if(err != NO_ERR)
		{
			//logerror(err)
		}

	#endif
	#ifdef HEX
		/*err = MotorDriver_setSpeed(uint8_t speed, Motor_Front_Left);
		if(err != NO_ERR)
			//logerror(err)
		err = MotorDriver_setSpeed(uint8_t speed, Motor_Front_Right);
		if(err != NO_ERR)
			//logerror(err)
		err = MotorDriver_setSpeed(uint8_t speed, Motor_Middle_Left);
		if(err != NO_ERR)
			//logerror(err)
		err = MotorDriver_setSpeed(uint8_t speed, Motor_Middle_Right);	
		if(err != NO_ERR)
			//logerror(err)		
		err = MotorDriver_setSpeed(uint8_t speed, Motor_Back_Left);
		if(err != NO_ERR)
			//logerror(err)
		err = MotorDriver_setSpeed(uint8_t speed, Motor_Back_Right);
		if(err != NO_ERR)
			//logerror(err)	*/
	#endif

}
