#pragma one
#include "Arduino.h"

// Define the pins
#define button 13
#define potentiometer 35
#define yellowLED 12
#define redLED 27
#define RgbLED 25
#define rGbLED 33
#define rgBLED 32

// RFID
#define SS_PIN 5
#define RST_PIN 16

// color for the screen
#define TFT_BORDEAU 0xC121  

// State variable :
// 0 = waiting for bluetooth connection, 
// 10 = waiting for rfid scan, 
// 20 = waiting for rfid tag name, 
// 30 = waiting for number selection, 
// 40 = waiting for confirmation.
extern int state; 


