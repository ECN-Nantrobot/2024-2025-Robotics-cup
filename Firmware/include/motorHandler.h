#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include "Arduino.h"

void initMotor();
void initialize_Z_axis();
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void allRunSpeed(void *pvParameters);
void moveAxeZ(int position, bool wait);
void go(int distance);

#endif
