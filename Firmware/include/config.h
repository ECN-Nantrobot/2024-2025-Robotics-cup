#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Wire.h"


// Define pins for motors
#define stepPin1 26
#define dirPin1 25
#define stepPin2 14
#define dirPin2 27

#define stepPinX 33
#define dirPinX 32


// Define pins for pump
#define pumpPin 15
#define vanPin 2
#define buttonPin1 4
#define buttonPin2 13
#define buttonPin3 21
#define buttonPinX 34

// Define pins for i2c
#define displaySDA 5
#define displaySLC 17

#define SDAPin 22
#define SLCPin 23

// Define motor interface type for AccelStepper
#define motorInterfaceType 1

// Dimensions of robot
const float wheelDiameter = 0.0675;   // meters
const float trackWidth = 0.185;       // meters
const int stepsPerRevolution = 1600;

// robot position
extern volatile float robotX;
extern volatile float robotY;
extern volatile float robotTheta;
extern volatile float _robotX;
extern volatile float _robotY;
extern volatile float _robotTheta;

extern SemaphoreHandle_t robotXMutex;
extern SemaphoreHandle_t i2cMutex;

// I2C instances
extern TwoWire myWire;
extern TwoWire wireDisplay;

// display instance
#include "displayHandler.h"
extern ecn::DisplayHandler display;


// control panel variable
extern bool isTestInProgress;
extern int testCurrentStep;



#endif