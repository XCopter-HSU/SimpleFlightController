#include <stdlib.h>

#include "PIDYaw.h"
#include "PIDConfig.h"

float PIDYawCalculation(float setpoint, float actualPosition) {

	//error state 
	static float priorError = 0;
	static float integral = 0;
	
	float error = setpoint - actualPosition;;
	float derivative = 0;
	float output = 0;
			
	//only integrate if error is big enough
	if(abs(error) >= YawEpsilon){
		//calculate integrative part
		integral += error * dT;	
	}
	
	//calculate the derivative part
	derivative = (error - priorError) / dT;
	
	//output calculation
	output = YawKp * error + YawKi * integral + YawKd * derivative;
	
	//limit the output maximum and minimum
	if(output > YawMaxOut) {
		output = YawMaxOut;
	}
	else if(output < YawMinOut) {
		output = YawMinOut;
	}
	
	//Update prior error
	priorError = error;
	
	return output;
}


/**
 * BIN GLEICH WIEDER DA by flo
 */
