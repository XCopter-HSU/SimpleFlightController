#ifndef PIDToMotorMapper__
#define PIDToMotorMapper__

#include <stdint.h>
#include "config.h"


int motorQuadx[4];
uint8_t motorHex[6];
float pidValues[4];


float computeCThrottle(float limitedThrottle, float CMix, float pidMix);

float PIDMIX(int8_t X, int8_t Y, int8_t Z, float throttle, float roll, float pitch, float yaw);

int8_t mapToMotors(float throttle, float roll, float pitch, float yaw);

void writeToMotors();

#endif
