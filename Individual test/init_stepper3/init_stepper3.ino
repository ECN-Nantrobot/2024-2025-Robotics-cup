#include <AccelStepper.h>

int end_pin = 4;
int dir_pin = 32;
int step_pin = 33;
AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);
int end_sensor_state = LOW;

void initialize() {
  // Go to end switch
  stepper.setSpeed(-20000);
  
  while (end_sensor_state == LOW) {
    end_sensor_state = digitalRead(end_pin);
    stepper.runSpeed();
  }

  // Move to a security margin
  stepper.move(100);
  while (stepper.currentPosition() != stepper.targetPosition()) {
    stepper.run();
  }

  // Set the default position
  stepper.setCurrentPosition(0);
}

void setup() {
  Serial.begin(115200);
  pinMode(end_pin, INPUT);
  stepper.setAcceleration(2000000);
  stepper.setMaxSpeed(20000);
  delay(100);
  initialize();
  stepper.move(200000);
}

void loop() {
  Serial.println(stepper.run());
}
