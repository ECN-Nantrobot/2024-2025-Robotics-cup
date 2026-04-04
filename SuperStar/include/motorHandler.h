#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include "Arduino.h"

bool isBlueTeam();
void initMotor();
void setMotorTarget(float target);
void setTargetAngle(float angle);
void pauseMotors();
void resumeMotors();
bool isTargetReached();
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void allRunSpeed(void *pvParameters);

#endif
