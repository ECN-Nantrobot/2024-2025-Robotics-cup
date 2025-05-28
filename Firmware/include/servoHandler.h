#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include "Arduino.h"

void initServo();
void setServo(int servo, int value);
void testServo();

#endif