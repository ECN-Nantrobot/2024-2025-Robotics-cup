#pragma one
#include "Arduino.h"

// Define the pins
#define LED_BUILTIN 2

// Nema driver pins
const int dirPin1 = 25;  // Motor 1 direction pin
const int stepPin1 = 26; // Motor 1 step pin
const int dirPin2 = 32;  // Motor 2 direction pin
const int stepPin2 = 33; // Motor 2 step pin

// Define motor interface type
#define motorInterfaceType 1