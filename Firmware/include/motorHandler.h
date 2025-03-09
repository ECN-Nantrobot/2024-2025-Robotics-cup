#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include "Arduino.h"

void initMotor();
void setMotorSpeeds(float left_speed_to_set, float right_speed_to_set);
void allRunSpeed(void *pvParameters);


#endif