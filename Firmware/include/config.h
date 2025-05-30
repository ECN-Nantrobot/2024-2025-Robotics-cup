#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Wire.h"
// #include "displayHandler.h"

// Define pins for motors
#define stepPin1 26
#define dirPin1 25
#define stepPin2 14
#define dirPin2 27

// Define pins for i2c
#define displaySDA 19
#define displaySLC 21

#define SDAPin 22
#define SLCPin 23

#define stepPinX 33
#define dirPinX 32

// Define pins for pump
#define pumpPin 15
#define vanPin 2
#define buttonPin3 21
// #define buttonPinX 34
// #define buttonPin1 4
// #define buttonPin2 13
#define starter_button 35
#define colour_button 13
#define save_start_button 12

// Define motor interface type for AccelStepper
#define motorInterfaceType 1

// Dimensions of robot
const float wheelDiameter = 0.0804;   // meters
const float trackWidth = 0.376;       // meters
const int stepsPerRevolution = 200;

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


// Forward declaration of ecn::DisplayHandler
// namespace ecn {
// 	class DisplayHandler;
// }

// display instance
// #include "displayHandler.h"
// extern ecn::DisplayHandler display;

// control panel variable
extern bool isTestInProgress;
extern int testCurrentStep;
extern bool starter; 
extern bool is_blue; 

#endif