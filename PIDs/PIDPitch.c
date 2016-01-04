#include <stdlib.h>

#include "PIDConfig.h"
#include "PIDPitch.h"



float PIDPitchCalculation(float setpoint, float actualPosition)
{

	//error state 
	static float priorError = 0;
	static float integral = 0;
	
	float error = setpoint - actualPosition;;
	float derivative = 0;
	float output = 0;
			
	//only integrate if error is big enough
	if(abs(error) >= PitchEpsilon){
		//calculate integrative part
		integral += error * dT;	
	}
	
	//calculate the derivative part
	derivative = (error - priorError) / dT;
	
	//output calculation
	output = PitchKp * error + PitchKi * integral + PitchKd * derivative;
	
	//limit the output maximum and minimum
	if(output > PitchMaxOut) {
		output = PitchMaxOut;
	}
	else if(output < PitchMinOut) {
		output = PitchMinOut;
	}
	
	//Update prior error
	priorError = error;
	
	return output;
}
