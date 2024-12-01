#include "BluetoothHandler.h"
#include "BluetoothSerial.h"
#include "DisplayHandler.h"

#include "SerialANSI.h"
#include "Utils.h"
#include "Config.h"

BluetoothSerial SerialBT;

void initBluetooth(String device_name) {
  SerialBT.begin(device_name);
  Serial.print(SerialFAINT);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  Serial.print(SerialRESET);
}



// Handles Bluetooth Commands from the python software
void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String receivedCommand = SerialBT.readStringUntil('\n');  // Read incoming Bluetooth serial data
    receivedCommand.trim();  // Remove any extra spaces or newlines

    if(receivedCommand == "BT-Connection"){
        state = 10;
        displayState("RFID tag");
    }else if(receivedCommand == "Value received"){
        state = 10; 
        displayState("RFID tag");
    }else{
        displayName(receivedCommand);
        state = 30;
    }
  }
}

// Send the RFID tag UID to the Bluetooth-connected device.
void sendRFIDUID(String uid) {
  SerialBT.print("RFID UID: ");
  SerialBT.println(uid);  // Send the UID via Bluetooth
  Serial.println(uid);    // Also print to the serial monitor
}

void sendValue(int value) {
    SerialBT.print("Value: ");
    SerialBT.println(value);  // Send the UID via Bluetooth
}
