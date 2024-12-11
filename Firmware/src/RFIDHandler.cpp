#include "RFIDHandler.h"
#include <MFRC522.h>
#include "BluetoothHandler.h"
#include "Config.h"
#include "SerialANSI.h"
#include "Utils.h"
#include "DisplayHandler.h"

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
byte nuidPICC[4];  // Stores the NUID of the RFID tag

void initRFID(byte ss_pin, byte rst_pin) {
  // Initialize RFID reader
  SPI.begin();  // Init SPI bus
  rfid.PCD_Init();  // Init MFRC522 
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;  // Default key value for RFID
  }
  SerialLog("RFID scanner READY");
}

// Handles RFID tag scanning and sending UID via Bluetooth
void handleRFIDScan() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.println(F("RFID Tag detected."));
    sendRFIDUID(getRFIDUID());  // Send the RFID tag UID via Bluetooth
    rfid.PICC_HaltA();  // Halt PICC
    rfid.PCD_StopCrypto1();  // Stop encryption on PCD

    state = 20;
    displayState("article");
  }
}

String getRFIDUID(){
  Serial.print(F("The NUID tag is: "));
  Serial.print(F("In hex: "));
  printHex(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  // Convert the UID to a string and send via Bluetooth
  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uidStr += String(rfid.uid.uidByte[i], HEX);
    if (i != rfid.uid.size - 1) {
      uidStr += ":";
    }
  }
    return uidStr;
}