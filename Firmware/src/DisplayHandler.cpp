#include "DisplayHandler.h"
#include <TFT_eSPI.h>
#include "Config.h"

#include "SerialANSI.h"


// TFT display
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

// fonts
int numberFont = 9;
int itemFont = 2;
int stateFont = 3;

// Setup the TFT screen
void initDisplay() {
  tft.init();
  tft.setRotation(0);

  tft.fillScreen(TFT_BORDEAU);

  tft.fillCircle(120, 120, 110, TFT_BLACK);

  tft.setTextColor(TFT_WHITE);

  tft.setTextSize(4);

  tft.setTextDatum(MC_DATUM);
  tft.drawString("ReFillIt", tft.width()/2, tft.height()/2);
  SerialLog("screen ready");
  delay(1000);
}


// Update the TFT screen with the mapped potentiometer value
void displayValue(int value) {
  // Draw a rectangle to hide the previous number
  int rectWidth = 200;
  int rectX = (tft.width()-rectWidth)/2;
  int rectY = (tft.height()-numberFont*8)/2;
  tft.fillRect(rectX, rectY, rectWidth, numberFont*8, TFT_BLACK);

  // Print the mapped value
  tft.setTextSize(numberFont);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(String(value), tft.width()/2, tft.height()/2);
}

void displayName(String name) {
    int offsetCenter = (numberFont/2 + itemFont/2)*8 + 10;
    resetScreen();
  // Print the mapped value
  tft.setTextSize(itemFont);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(name, tft.width()/2, tft.height()/2+offsetCenter);
}

void displayState(String name){
    resetScreen();

  // Print the mapped value
  tft.setTextSize(stateFont);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(name, tft.width()/2, tft.height()/2);
  tft.drawString("waiting", tft.width()/2, tft.height()/2-stateFont*8/2-30);

}

void resetScreen(){
  tft.fillScreen(TFT_BORDEAU);

  tft.fillCircle(120, 120, 110, TFT_BLACK);
}