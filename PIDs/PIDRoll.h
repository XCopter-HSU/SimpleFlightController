#ifndef PID_ROLL_H_
#define PID_ROLL_H_
#include <math.h>

/*
 * PID Factors
 */

#define RollKp 4.0
#define RollKi 0.0
#define RollKd 10.0

/*
 * PID maximum and minimum output
 */
#define RollMaxOut 20
#define RollMinOut -20
 
//minimum error for integration
#define RollEpsilon 0.05f


float PIDRollCalculation(float setpoint, float actualPosition);

#endif
