#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include "Arduino.h"

void initMotor();
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void allRunSpeed(void *pvParameters);

#endif
