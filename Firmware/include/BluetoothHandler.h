#pragma one
#include "Arduino.h"

void initBluetooth(String device_name);
void handleBluetoothCommands();
void sendRFIDUID(String uid);
void sendValue(int value);