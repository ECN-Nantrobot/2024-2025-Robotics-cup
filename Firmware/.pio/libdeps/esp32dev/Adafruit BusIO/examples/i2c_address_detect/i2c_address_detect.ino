#include <Adafruit_I2CDevice.h>

Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(0x10);

void setup() {
<<<<<<< HEAD
  while (!Serial) { delay(10); }
=======
  while (!Serial) {
    delay(10);
  }
>>>>>>> main
  Serial.begin(115200);
  Serial.println("I2C address detection test");

  if (!i2c_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
<<<<<<< HEAD
    while (1);
=======
    while (1)
      ;
>>>>>>> main
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);
}

<<<<<<< HEAD
void loop() {
  
}
=======
void loop() {}
>>>>>>> main
