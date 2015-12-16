#include "PIDRoll.h"
#include "PIDConfig.h"
#include "stdlib.h"

float PIDRollCalculation(float setpoint, float actualPosition) {

	//error state 
	static float priorError = 0;
	static float integral = 0;
	
	float error = setpoint - actualPosition;;
	float derivative = 0;
	float output = 0;
			
	//only integrate if error is big enough
	if(abs(error) >= RollEpsilon){
		//calculate integrative part
		integral += error * dT;	
	}
	
	//calculate the derivative part
	derivative = (error - priorError) / dT;
	
	//output calculation
	output = RollKp * error + RollKi * integral + RollKd * derivative;
	
	//limit the output maximum and minimum
	if(output > RollMaxOut) {
		output = RollMaxOut;
	}
	else if(output < RollMinOut) {
		output = RollMinOut;
	}
	
	//Update prior error
	priorError = error;
	
	return output;
}
