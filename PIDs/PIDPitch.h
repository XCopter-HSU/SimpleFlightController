#ifndef PID_PITCH_H_
#define PID_PITCH_H_

#include <math.h>

/*
 * PID Factors
 */
#define PitchKp 4.0
#define PitchKi 0.0
#define PitchKd 10.0
	
/*
 * PID maximum and minimum output
 */
#define PitchMaxOut 20
#define PitchMinOut -20
 
//minimum error for integration
#define PitchEpsilon 0.05f

//Pitch PID
float PIDPitchCalculation(float setpoint, float actualPosition);

#endif
