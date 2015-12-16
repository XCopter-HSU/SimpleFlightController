#ifndef PID_PITCH_H_
#define PID_PITCH_H_

#include <math.h>

/*
 * PID Factors
 */
#define PitchKp 4.5
#define PitchKi 0.1
#define PitchKd 0
	
/*
 * PID maximum and minimum output
 */
#define PitchMaxOut 65535
#define PitchMinOut -65535
 
//minimum error for integration
#define PitchEpsilon 0.05f

//Pitch PID
float PIDPitchCalculation(float setpoint, float actualPosition);

#endif
