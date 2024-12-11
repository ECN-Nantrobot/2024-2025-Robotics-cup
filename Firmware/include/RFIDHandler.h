#pragma one
#include "Arduino.h"

void initRFID(byte ss_pin, byte rst_pin);
void handleRFIDScan();
String getRFIDUID();