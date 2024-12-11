#include "config.h"
#include "motorHandler.h"


#include "SerialANSI.h"


void setup() { 
  // Initialize Serial, Bluetooth, LED, and RFID
  Serial.begin(9600);

  Serial.println();
  SerialTitle("=== Welcome to the test of the mobile test ===");
  Serial.println();

  initMotor(); 

}


void loop() {
  testMotor();
}










