#include "debug.h"

// Active/désactive les logs standards
bool debug = true;

// Codes ANSI nécessaires
String SerialRESET = "\033[0m";
String SerialBOLD = "\033[1m";
String SerialFAINT = "\033[2m";
String SerialITALIC = "\033[3m";
String SerialUNDERLINE = "\033[4m";

String SerialFG_RED = "\033[31m";
String SerialFG_GREEN = "\033[32m";
String SerialFG_YELLOW = "\033[33m";
String SerialFG_CYAN = "\033[36m";

String SerialBG_RED = "\033[41m";

// Fonction pour log standard (désactivable)
void SerialLog(String message) {
  if (!debug) return;
  Serial.print(SerialFAINT);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Message important (texte rouge + gras)
void SerialImportant(String message) {
  Serial.print(SerialFG_RED);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Message critique (fond rouge + gras)
void SerialCritical(String message) {
  Serial.print(SerialBG_RED);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Message succès (vert + gras)
void SerialSuccess(String message) {
  Serial.print(SerialFG_GREEN);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Message avertissement (jaune + gras)
void SerialWarning(String message) {
  Serial.print(SerialFG_YELLOW);
  Serial.print(SerialBOLD);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Message info (cyan + italique)
void SerialInfo(String message) {
  Serial.print(SerialFG_CYAN);
  Serial.print(SerialITALIC);
  Serial.print(message);
  Serial.println(SerialRESET);
}

// Titre centré (remplace l'ancien SerialTitleCentered)
void SerialTitle(String message) {
  Serial.println();
  int messageLength = message.length();
  int paddingLength = (SERIAL_WIDTH - messageLength - 2) / 2; // 2 for spaces

  String padding = "";
  for (int i = 0; i < paddingLength; i++) {
    padding += "=";
  }

  Serial.println(padding + " " + message + " " + padding);

  if ((paddingLength * 2 + messageLength + 2) < SERIAL_WIDTH) {
    Serial.print("=");
  }
  Serial.println();
}

// Réinitialise le style du terminal
void SerialReset() {
  Serial.print(SerialRESET);
}

// Efface la ligne courante
void SerialClearCurLine() {
  Serial.print("\033[2K");
  Serial.print("\033[0G");
}

// Efface la ligne précédente
void SerialClearPrevLine() {
  Serial.print("\033[1A");
  Serial.print("\033[2K");
}

// Efface tout le terminal série
void SerialClearSerial() {
  Serial.print("\033[2J");
  Serial.print("\033[H");
}
