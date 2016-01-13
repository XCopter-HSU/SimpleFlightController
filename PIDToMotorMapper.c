#include <stdint.h>

#include "PIDToMotorMapper.h"
#include "Drivers/Driver_Motor.h"
#include "Errorcodes.h"


float computeCThrottle(float limitedThrottle, float CMix, float pidMix){
	//Compute the CThrottle value subject to CMix

	float CThrottle = (float) (100.0 / (limitedThrottle + CMix * pidMix));
	return CThrottle;
}

/*PIDMIX calculates the value for each motor.
Each motor gets a roll, pitch and yaw value from the pids and also a throttle value from
the remote control. The full speed value of throttle is limited to a maximal value of 75%, so 
there are +- 25% reserved for roll and pitch. Roll pitch and yaw are multiplied with the 
x, y and z value of the mapping table. 
*/ 
float PIDMIX(int8_t X, int8_t Y, int8_t Z, float throttle, float roll, float pitch, float yaw) {
	uint8_t CMixIsCorrect = 0;
	float CMix = 1.0;
	//limit the throttle to 75%
	float limitedThrottle = throttle * 0.75;
	float pidMix = 0.0;
	float throttleMix = limitedThrottle;

	while(CMixIsCorrect == 0) {
		//Compute the values using the mapping table
		pidMix = CMix * ((roll * X) + (pitch * Y) + (yaw * Z));
		throttleMix = limitedThrottle * computeCThrottle(limitedThrottle, CMix, pidMix);	
	
		if(pidMix > limitedThrottle) {
			CMix -= CMix > 0.1 ? 0.1 : 0;
		} else {
			CMixIsCorrect = 1;
		}	
	}	
	return  throttleMix + pidMix; 
} 
								


//Maps all four values to each motor and computes the the PWM- signal for the motors.
int8_t mapToMotors(float throttle, float roll, float pitch, float yaw){
	
	if((roll < -25) || (pitch < -25) || (yaw < -25)){
		//error auserhalb der range
		return ERR_MOTORMAPPER_PID_ILLEGAL_RANGE;
	}
	
	if((roll > 25) || (pitch > 25) || (yaw > 25)){
		//error auserhalb der range
		return ERR_MOTORMAPPER_PID_ILLEGAL_RANGE;
	}
	
/*Maps the range of the pid values and the range of the throttle value into the range of the
PWM signal*/		
	
	#ifdef QUADX	
		//Mapping table for a QUADX configuration
		motorQuadx[0] = PIDMIX(-1,+1,-1, throttle,roll, pitch, yaw); //REAR_R
		motorQuadx[1] = PIDMIX(-1,-1,+1, throttle,roll, pitch, yaw); //FRONT_R
		motorQuadx[2] = PIDMIX(+1,+1,+1, throttle,roll, pitch, yaw); //REAR_L
		motorQuadx[3] = PIDMIX(+1,-1,-1, throttle,roll, pitch, yaw); //FRONT_L
		
		//motorQuadx in PWM umrechnen--> evtl PWM nur in % umrechnen und prozent-treiber benutzen
		printf("M1:%d\tM2:%d\tM3:%d\tM4:%d\n", motorQuadx[0], motorQuadx[1], motorQuadx[2], motorQuadx[3]);
		
		
	#endif
	#ifdef HEX
//		motorHex[0] = PIDMIX(-1,+1,-1, throttle,roll, pitch, yaw);															//	----TODO----
//		motorHex[1] = PIDMIX(-1,-1,+1, throttle,roll, pitch, yaw);
//		motorHex[2] = PIDMIX(+1,+1,+1, throttle,roll, pitch, yaw);
//		motorHex[3] = PIDMIX(+1,-1,-1, throttle,roll, pitch, yaw);
//		motorHex[4] = PIDMIX(+1,-1,-1, throttle,roll, pitch, yaw);
//		motorHex[5] = PIDMIX(+1,-1,-1, throttle,roll, pitch, yaw);
	 #endif
	 
	writeToMotors();
	//TODO errors
	return NO_ERR;
 }
 

//Writes the PWM- signal to each motor. 
int8_t writeToMotors()
{	
	int16_t err = NO_ERR;	
	#ifdef QUADX
		err = MotorDriver_setSpeedPercent(motorQuadx[3], Motor_Front_Left);							
		if(err != NO_ERR)
		{
			return err;
		}

		err = MotorDriver_setSpeedPercent(motorQuadx[1], Motor_Front_Right);
		if(err != NO_ERR)
		{
			return err;
		}

		err = MotorDriver_setSpeedPercent(motorQuadx[2], Motor_Back_Left);
		if(err != NO_ERR)
		{
			return err;
		}

		err = MotorDriver_setSpeedPercent(motorQuadx[0], Motor_Back_Right);
		if(err != NO_ERR)
		{
			return err;
		}

	#endif
	#ifdef HEX
		//TODO
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
