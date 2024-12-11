#include "Config.h"
#include "BluetoothHandler.h"
#include "RFIDHandler.h"
#include "DisplayHandler.h"
#include "InputHandler.h"
#include "Utils.h"

#include "SerialANSI.h"

// Variables
String device_name = "ESP32-BT-ReFillIt";
int state = 0; // initialize state to waiting rfid tag

void setup() { 
  // Initialize Serial, Bluetooth, LED, and RFID
  Serial.begin(9600);

  Serial.println();
  SerialTitle("=== Welcome to ReFillIt ===");
  Serial.println();
  SerialInfo("An IoT school project for effortless restocking using RFID.");
  SerialInfo("Tap. Select Quantity. Confirm.");
  Serial.println();

  initBluetooth(device_name);
  initRFID(SS_PIN, RST_PIN);
  initDisplay();
  initInputs();

  Serial.println();
  SerialSuccess("RefillIt is now ready to use!");

  displayState("BT");
  while (state==0){
    handleBluetoothCommands();
    handleLED();
    delay(100); 
  }
}

int potentiometerValue;

void loop() {
  potentiometerValue = -1;
  while (state == 10){
    handleRFIDScan();
    handleLED();
    delay(100);
  }
  while (state == 20){
    handleBluetoothCommands();
    handleLED();
    delay(100);
  }
  while (state == 30){
    potentiometerValue = handlePotentiometerReading(potentiometerValue);
    handleButton(potentiometerValue);
    handleRFIDScan();
    handleLED();
    delay(100);
  }
  
  while (state == 40){
    handleBluetoothCommands();
    handleLED();
    delay(100);
  }

  delay(100);  // Small delay to avoid flooding the loop
}










