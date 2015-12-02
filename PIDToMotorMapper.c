#include "PIDToMotorMapper.h"
#include "Drivers/Driver_Motor.h"
#include "Errorcodes.h"

/*PIDMIX calculates the value for each motor.
Each motor gets a roll, pitch and yaw value from the pids and also a throttle value from
the remote control. The full speed value of throttle is limited to a maximal value of 75%, so 
there are +- 25% reserved for roll and pitch. Roll pitch and yaw are multiplied with the 
x, y and z value of the mapping table. 
*/ 
#define PIDMIX(X,Y,Z) (throttle * 0.75) + roll *X + pitch *Y + yaw *Z    										


//Maps all four values to each motor and computes the the PWM- signal for the motors.
int8_t mapToMotors(float throttle, float roll, float pitch, float yaw){
	#ifdef QUADX
		//Mapping table for a QUADX configuration
		motorQuadx[0] = PIDMIX(-1,+1,-1); //REAR_R
		motorQuadx[1] = PIDMIX(-1,-1,+1); //FRONT_R
		motorQuadx[2] = PIDMIX(+1,+1,+1); //REAR_L
		motorQuadx[3] = PIDMIX(+1,-1,-1); //FRONT_L	
		
		//Compute PWM- signal and write to Motors
		int8_t i;
		for(i = 0; i < sizeof(motorQuadx); i++){
			motorQuadx[i] = motorQuadx[i] * scaleToPWM;
		}
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
	
	return NO_ERR;
 }
 

//Writes the PWM- signal to each motor. 
void writeToMotors()
{
	
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
