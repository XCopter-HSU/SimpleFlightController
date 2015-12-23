#ifndef PID_YAW_H_
#define PID_YAW_H_
#include <math.h>

/*
 * PID Factors
 */
#define YawKp 4.5
#define YawKi 0.0
#define YawKd 8.0
	
/*
 * PID maximum and minimum output
 */
#define YawMaxOut 20
#define YawMinOut -20
 
//minimum error for integration
#define YawEpsilon 0.05f

float PIDYawCalculation(float setpoint, float actualPosition);

#endif
