#include "Config.h"
#include "servoHandler.h"
#include <Wire.h> 
#include <Adafruit_PWMServoDriver.h>


#define SERVOMIN 250  // Minimum pulse length
#define SERVOMAX 800  // Maximum pulse length

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

int angleToPulse(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: "); Serial.print(ang);
    Serial.print(" pulse: "); Serial.println(pulse);
    return pulse;
}


void initServo() {
    Wire.begin(SDAPin, SLCPin); // Initialize I2C with custom pins
    board1.begin();
    board1.setPWMFreq(60);  // Set PWM frequency for servos
    Serial.begin(9600);
    Serial.println("Servo driver initialized.");


    pinMode(buttonPin2, INPUT_PULLDOWN);

  xTaskCreatePinnedToCore(
    testServo,   // Task function
    "testServo", // Name
    2048,          // Stack size
    NULL,          // Parameters
    1,             // Lower priority (instead of a high value)
    NULL,          // Task handle
    0              // CPU core (1)
  );
    
}


void setServo(int servo, int value) {
    if (value < 0) value = 0;
    if (value > 180) value = 110;
    Serial.print("Set servo "); Serial.print(servo);Serial.print(" to "); Serial.println(value);
    board1.setPWM(servo, 0, angleToPulse(value));
}

bool activated2 = false;
bool inProcess2 = false;
void testServo(void *pvParameters) {
    while (1){
        if(digitalRead(buttonPin2)&&!inProcess2)
        {
            inProcess2 = true;
            Serial.println("button2Pressed");
            if(activated2){
                setServo(0, 0);
                activated2=!activated2;
            }else{
                setServo(0, 180);
                activated2=!activated2;
            }

        }
        if(!digitalRead(buttonPin2)){
            inProcess2 = false;
        }
        delay(200);
    }
}