#ifndef PID_ROLL_H_
#define PID_ROLL_H_
#include <math.h>

/*
 * PID Factors
 */

#define RollKp 4.5
#define RollKi 0.1
#define RollKd 0

/*
 * PID maximum and minimum output
 */
#define RollMaxOut 65536
#define RollMinOut -65535
 
//minimum error for integration
#define RollEpsilon 0.05f


float PIDRollCalculation(float setpoint, float actualPosition);

#endif
