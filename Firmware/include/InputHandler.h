#pragma once
#include "Arduino.h"

void initInputs();
int handlePotentiometerReading(int prevValue);
void handleButton(int value);
void handleLED();
void setRGBColor(int red, int green, int blue);