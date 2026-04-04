#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H
#include "Arduino.h"

void initMotor();
<<<<<<< HEAD
void setMotorSpeeds(float left_speed_to_set, float right_speed_to_set);
void allRunSpeed(void *pvParameters);

void initialize_Z_axis();
void moveAxeZ(int position, bool wait);
void go(int distance);
#endif
=======
void initialize_Z_axis();
void setMotorSpeeds(float leftSpeed, float rightSpeed);
void allRunSpeed(void *pvParameters);
void moveAxeZ(int position, bool wait);
void go(int distance);

#endif
>>>>>>> main
