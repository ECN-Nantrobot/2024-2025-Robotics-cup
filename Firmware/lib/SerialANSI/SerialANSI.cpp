// SerialANSI.cpp
#include "SerialANSI.h"

// Variable Definitions
String SerialRESET = "\033[0m";
String SerialBOLD = "\033[1m";
String SerialFAINT = "\033[2m";
String SerialITALIC = "\033[3m";
String SerialUNDERLINE = "\033[4m";
String SerialREVERSE = "\033[7m";
String SerialCONCEAL = "\033[8m";
String SerialCROSSED_OUT = "\033[9m";
String SerialDOUBLE_UNDERLINE = "\033[21m";
String SerialNORMAL_INTENSITY = "\033[22m";
String SerialNOT_ITALIC = "\033[23m";
String SerialUNDERLINE_OFF = "\033[24m";
String SerialBLINK_OFF = "\033[25m";
String SerialINVERSE_OFF = "\033[27m";
String SerialREVEAL = "\033[28m";
String SerialNOT_CROSSED = "\033[29m";

// Foreground Colors (30-37)
String SerialFG_BLACK = "\033[30m";
String SerialFG_RED = "\033[31m";
String SerialFG_GREEN = "\033[32m";
String SerialFG_YELLOW = "\033[33m";
String SerialFG_BLUE = "\033[34m";
String SerialFG_MAGENTA = "\033[35m";
String SerialFG_CYAN = "\033[36m";
String SerialFG_WHITE = "\033[37m";

// Background Colors (40-47)
String SerialBG_BLACK = "\033[40m";
String SerialBG_RED = "\033[41m";
String SerialBG_GREEN = "\033[42m";
String SerialBG_YELLOW = "\033[43m";
String SerialBG_BLUE = "\033[44m";
String SerialBG_MAGENTA = "\033[45m";
String SerialBG_CYAN = "\033[46m";
String SerialBG_WHITE = "\033[47m";

// Default Colors (39, 49)
String SerialFG_DEFAULT = "\033[39m";
String SerialBG_DEFAULT = "\033[49m";

// Additional Styles
String SerialOVERLINED = "\033[53m";
String SerialNOT_OVERLINED = "\033[55m";

// Ideogram Styles
String SerialIDEOGRAM_UNDERLINE = "\033[60m";
String SerialIDEOGRAM_DOUBLE_UNDERLINE = "\033[61m";
String SerialIDEOGRAM_OVERLINE = "\033[62m";
String SerialIDEOGRAM_DOUBLE_OVERLINE = "\033[63m";
String SerialIDEOGRAM_STRESS_MARKING = "\033[64m";
String SerialIDEOGRAM_OFF = "\033[65m";

// Bright Foreground Colors (90-97)
String SerialBRIGHT_FG_BLACK = "\033[90m";
String SerialBRIGHT_FG_RED = "\033[91m";
String SerialBRIGHT_FG_GREEN = "\033[92m";
String SerialBRIGHT_FG_YELLOW = "\033[93m";
String SerialBRIGHT_FG_BLUE = "\033[94m";
String SerialBRIGHT_FG_MAGENTA = "\033[95m";
String SerialBRIGHT_FG_CYAN = "\033[96m";
String SerialBRIGHT_FG_WHITE = "\033[97m";

// Bright Background Colors (100-107)
String SerialBRIGHT_BG_BLACK = "\033[100m";
String SerialBRIGHT_BG_RED = "\033[101m";
String SerialBRIGHT_BG_GREEN = "\033[102m";
String SerialBRIGHT_BG_YELLOW = "\033[103m";
String SerialBRIGHT_BG_BLUE = "\033[104m";
String SerialBRIGHT_BG_MAGENTA = "\033[105m";
String SerialBRIGHT_BG_CYAN = "\033[106m";
String SerialBRIGHT_BG_WHITE = "\033[107m";

// Function Definitions
void SerialLog(String message) {
  Serial.print(SerialFAINT);
  Serial.print(message);
  Serial.println(SerialRESET); // Reset styles after printing
}

void SerialImportant(String message) {
  Serial.print(SerialFG_RED);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialCritical(String message) {
  Serial.print(SerialBG_RED);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialSuccess(String message) {
  Serial.print(SerialFG_GREEN);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialWarning(String message) {
  Serial.print(SerialFG_YELLOW);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialInfo(String message) {
  Serial.print(SerialFG_CYAN);
  Serial.print(SerialITALIC);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialDebug(String message) {
  Serial.print(SerialFG_MAGENTA);
  Serial.print(SerialFAINT);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialTitle(String message) {
  Serial.print(SerialUNDERLINE);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialHighlight(String message) {
  Serial.print(SerialBG_YELLOW);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

void SerialReset() {
  Serial.print(SerialRESET); // Clears any active styles
}

void SerialClearCurLine() {
  Serial.print("\033[2K");    // Clear the entire current line
  Serial.print("\033[0G");    // Move the cursor to the start of the current line
}

// Function to clear the previous line in the Serial Monitor
void SerialClearPrevLine() {
  Serial.print("\033[1A");    // Move the cursor up one line
  Serial.print("\033[2K");    // Clear the entire line
}

// Function to clear the Serial Monitor screen
void SerialClearSerial() {
  Serial.print("\033[2J");    // Clear the screen
  Serial.print("\033[H");     // Move cursor to the top-left corner
}

// Function to center and title a message
void SerialTitleCentered(String message) {
  int messageLength = message.length();
  int paddingLength = (SERIAL_WIDTH - messageLength) / 2;

  // Create the padding
  String padding = "";
  for (int i = 0; i < paddingLength; i++) {
    padding += "=";
  }

  // Print the formatted title
  Serial.println(padding + " " + message + " " + padding);

  // If the width is odd and there’s a character missing at the end, add one more '='
  if ((paddingLength * 2 + messageLength) < SERIAL_WIDTH) {
    Serial.print("=");
  }
  Serial.println();
}
