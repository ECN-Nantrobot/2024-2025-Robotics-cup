// SerialANSI.h
#pragma once
#include "Arduino.h"

// from https://github.com/py-Alexis/SerialANSI

// Variable Declarations
extern String SerialRESET;
extern String SerialBOLD;
extern String SerialFAINT;
extern String SerialITALIC;
extern String SerialUNDERLINE;
extern String SerialREVERSE;
extern String SerialCONCEAL;
extern String SerialCROSSED_OUT;
extern String SerialDOUBLE_UNDERLINE;
extern String SerialNORMAL_INTENSITY;
extern String SerialNOT_ITALIC;
extern String SerialUNDERLINE_OFF;
extern String SerialBLINK_OFF;
extern String SerialINVERSE_OFF;
extern String SerialREVEAL;
extern String SerialNOT_CROSSED;

// Foreground Colors (30-37)
extern String SerialFG_BLACK;
extern String SerialFG_RED;
extern String SerialFG_GREEN;
extern String SerialFG_YELLOW;
extern String SerialFG_BLUE;
extern String SerialFG_MAGENTA;
extern String SerialFG_CYAN;
extern String SerialFG_WHITE;

// Background Colors (40-47)
extern String SerialBG_BLACK;
extern String SerialBG_RED;
extern String SerialBG_GREEN;
extern String SerialBG_YELLOW;
extern String SerialBG_BLUE;
extern String SerialBG_MAGENTA;
extern String SerialBG_CYAN;
extern String SerialBG_WHITE;

// Default Colors (39, 49)
extern String SerialFG_DEFAULT;
extern String SerialBG_DEFAULT;

// Additional Styles
extern String SerialOVERLINED;
extern String SerialNOT_OVERLINED;

// Ideogram Styles
extern String SerialIDEOGRAM_UNDERLINE;
extern String SerialIDEOGRAM_DOUBLE_UNDERLINE;
extern String SerialIDEOGRAM_OVERLINE;
extern String SerialIDEOGRAM_DOUBLE_OVERLINE;
extern String SerialIDEOGRAM_STRESS_MARKING;
extern String SerialIDEOGRAM_OFF;

// Bright Foreground Colors (90-97)
extern String SerialBRIGHT_FG_BLACK;
extern String SerialBRIGHT_FG_RED;
extern String SerialBRIGHT_FG_GREEN;
extern String SerialBRIGHT_FG_YELLOW;
extern String SerialBRIGHT_FG_BLUE;
extern String SerialBRIGHT_FG_MAGENTA;
extern String SerialBRIGHT_FG_CYAN;
extern String SerialBRIGHT_FG_WHITE;

// Bright Background Colors (100-107)
extern String SerialBRIGHT_BG_BLACK;
extern String SerialBRIGHT_BG_RED;
extern String SerialBRIGHT_BG_GREEN;
extern String SerialBRIGHT_BG_YELLOW;
extern String SerialBRIGHT_BG_BLUE;
extern String SerialBRIGHT_BG_MAGENTA;
extern String SerialBRIGHT_BG_CYAN;
extern String SerialBRIGHT_BG_WHITE;

// Function Declarations
void SerialLog(String message);
void SerialImportant(String message);
void SerialCritical(String message);
void SerialSuccess(String message);
void SerialWarning(String message);
void SerialInfo(String message);
void SerialDebug(String message);
void SerialTitle(String message);
void SerialHighlight(String message);
void SerialReset();
void SerialClearCurLine();
void SerialClearPrevLine();
void SerialClearSerial();

// Define the width of the Serial Monitor
#define SERIAL_WIDTH 80

void SerialTitleCentered(String message);
