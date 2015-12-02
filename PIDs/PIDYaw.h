#ifndef PID_YAW_H_
#define PID_YAW_H_
#include <math.h>

/*
 * PID Factors
 */
const float YawKp = 0;
const float YawKi = 0;
const float YawKd = 0;
	
/*
 * PID maximum and minimum output
 */
 const float YawMaxOut = 0;
 const float YawMinOut = 0;
 
//minimum error for integration
const float YawEpsilon = 0.05;

float PIDYawCalculation(float setpoint, float actualPosition);

#endif
