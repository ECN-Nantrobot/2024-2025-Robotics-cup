#include "InputHandler.h"
#include "Config.h"
#include "BluetoothHandler.h"
#include "DisplayHandler.h"

void initInputs() {
    // Initialize PINS
    pinMode(button, INPUT); 
    pinMode(potentiometer, INPUT); 
    pinMode(yellowLED, OUTPUT); 
    pinMode(redLED, OUTPUT); 
    pinMode(RgbLED, OUTPUT); 
    pinMode(rGbLED, OUTPUT); 
    pinMode(rgBLED, OUTPUT); 
}

// Handles the potentiometer reading and updates the TFT screen
int handlePotentiometerReading(int prevValue) {
  // Read potentiometer value (ESP32: range from 0 to 4095)
  int potValue = analogRead(potentiometer);
  int mappedValue;

  // Map the analog value to custom ranges
  if (potValue <= 1365) {
    // Region 1: Map to values between 1 and 10
    mappedValue = map(potValue, 0, 1365, 1, 10);
    // Optionally, round to the nearest integer
    mappedValue = constrain(mappedValue, 1, 10);
  } 
  else if (potValue <= 2730) {
    // Region 2: Map to values between 10 and 100, step of 10 (10, 20, 30, ... 100)
    mappedValue = map(potValue, 1366, 2730, 10, 100);
    mappedValue = ((mappedValue + 5) / 10) * 10;  // Ensure step of 10
    mappedValue = constrain(mappedValue, 10, 100);
  } 
  else {
    // Region 3: Map to values between 100 and 1000, step of 100 (100, 200, 300, ... 1000)
    mappedValue = map(potValue, 2731, 4095, 100, 1000);
    mappedValue = ((mappedValue + 50) / 100) * 100;  // Ensure step of 100
    mappedValue = constrain(mappedValue, 100, 1000);
  }

  // Only update the screen if the mapped value has changed
  if (mappedValue != prevValue) {
    // Serial.print("Potentiometer Value: ");
    // Serial.print(potValue);
    // Serial.print(" => Mapped Value: ");
    // Serial.println(mappedValue);

    // Update the screen with the new mapped value
    displayValue(mappedValue);
  }
  return mappedValue;
}

bool prevPress;
void handleButton(int value){
    if (digitalRead(button) && !prevPress){
      prevPress  = true;
      sendValue(value);
      state = 40;
      displayState("sending...");
      delay(2000);
    }
    if (!digitalRead(button)){
        prevPress = false;
    }
};

void handleLED(){
    if (digitalRead(button)){
      digitalWrite(redLED, HIGH);
    }else{
      digitalWrite(redLED, LOW);
    }

    if(state==10){
      setRGBColor(255, 0, 0);
    }
    if(state==20){
      setRGBColor(0, 255, 0);
    }if(state==30){
      setRGBColor(0, 0, 255);
    }if(state==40){
      setRGBColor(0, 255, 255);
    }
}

void setRGBColor(int redValue, int greenValue, int blueValue) {
  // Ensure values are within 0-255
  redValue = constrain(redValue, 0, 255);
  greenValue = constrain(greenValue, 0, 255);
  blueValue = constrain(blueValue, 0, 255);

  if(redValue==0){
    digitalWrite(RgbLED, HIGH);
  }else{
    digitalWrite(RgbLED, LOW);
  }
  if(greenValue==0){
    digitalWrite(rGbLED, HIGH);
  }else{
    digitalWrite(rGbLED, LOW);
  }
  if(blueValue==0){
    digitalWrite(rgBLED, HIGH);
  }else{
    digitalWrite(rgBLED, LOW);
  }
}