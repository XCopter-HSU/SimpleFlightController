#ifndef PIDToMotorMapper__
#define PIDToMotorMapper__

#include <stdint.h>
#include "config.h"


int8_t motorQuadx[4];
int8_t motorHex[6];
const float scaleToPWM = 1.0;

int8_t mapToMotors(float throttle, float roll, float pitch, float yaw);

void writeToMotors();



#endif
