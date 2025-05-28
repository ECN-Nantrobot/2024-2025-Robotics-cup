#ifndef DEBUG_H
#include "Arduino.h"

// Définition de la largeur du terminal série
#define SERIAL_WIDTH 80

extern bool debug;

void SerialLog(String message);
void SerialImportant(String message);
void SerialCritical(String message);
void SerialSuccess(String message);
void SerialWarning(String message);
void SerialInfo(String message);
void SerialTitle(String message);

void SerialReset();
void SerialClearCurLine();
void SerialClearPrevLine();
void SerialClearSerial();

#endif