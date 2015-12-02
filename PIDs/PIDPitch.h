#ifndef PID_PITCH_H_
#define PID_PITCH_H_

#include <math.h>

/*
 * PID Factors
 */
const float PitchKp = 0;
const float PitchKi = 0;
const float PitchKd = 0;
	
/*
 * PID maximum and minimum output
 */
 const float PitchMaxOut = 0;
 const float PitchMinOut = 0;
 
//minimum error for integration
const float PitchEpsilon = 0.05;

//Pitch PID
float PIDPitchCalculation(float setpoint, float actualPosition);

#endif
