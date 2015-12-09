//flight model
#ifndef PIDCONFIG_H_
#define PIDCONFIG_H_

#define QUADX 1

//************ Define PIDToMotorMapper ************//
// PWM Range geht von 8 (Motor aus) - 218 (Motor speed max) --> 1,2ms - 1,92ms
#define PWM_UPPER_LIMIT  218
#define PWM_LOWER_LIMIT  8

//************ Define PID Sampling Rate ***********//
#define dT 1

#endif
