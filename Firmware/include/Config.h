#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Wire.h"


// Define pins for motors
#define stepPin1 26
#define dirPin1 25
#define stepPin2 33
#define dirPin2 32
#define stepPinX 14
#define dirPinX 27


// Define pins for i2c
#define displaySDA 19
#define displaySLC 21

#define SDAPin 23
#define SLCPin 22

// Define motor interface type for AccelStepper
#define motorInterfaceType 1

// Dimensions of robot
const float wheelDiameter = 0.0675;   // meters
const float trackWidth = 0.185;       // meters
const int stepsPerRevolution = 3200;

// robot position
extern volatile float robotX;
extern volatile float robotY;
extern volatile float robotTheta;
extern volatile float _robotX;
extern volatile float _robotY;
extern volatile float _robotTheta;

extern SemaphoreHandle_t robotXMutex;

// I2C instances
extern TwoWire myWire;
extern TwoWire wireDisplay;

// display instance
#include "displayHandler.h"
extern ecn::DisplayHandler display;


#endif