#include <Adafruit_PWMServoDriver.h>
#include "Wire.h"

TwoWire myWire(0);
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40, myWire);  // called this way, it uses the default address 0x40

#define SERVOMIN 190  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 800  // this is the 'maximum' pulse length count (out of 4096)




void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  myWire.begin(22, 23);
  if (board1.begin()) {
    Serial.println("succesful");
  } else {
    Serial.println("fuckkkkkkkkkkkkkkk");
    while (1) {}
  }
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
  for (int i = 0; i < 16; i++) { board1.setPWM(i, 0, angleToPulse(0)); }
  delay(3000);
  for (int i = 0; i < 16; i++) { board1.setPWM(i, 0, angleToPulse(90)); }
  delay(3000);
}

int angleToPulse(int ang)  //gets angle in degree and returns the pulse width
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}