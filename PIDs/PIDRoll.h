#ifndef PID_ROLL_H_
#define PID_ROLL_H_
#include <math.h>

/*
 * PID Factors
 */
const float RollKp = 0;
const float RollKi = 0;
const float RollKd = 0;
	
/*
 * PID maximum and minimum output
 */
 const float RollMaxOut = 0;
 const float RollMinOut = 0;
 
//minimum error for integration
const float RollEpsilon = 0.05;


float PIDRollCalculation(float setpoint, float actualPosition);

#endif
