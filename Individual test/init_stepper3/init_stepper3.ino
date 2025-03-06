#include <AccelStepper.h>

int end_pin = 5;
int dir_pin = 6;
int step_pin = 7;
AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);
int end_sensor_state = LOW;

void initialize() {
  // Go to end switch
  stepper.setSpeed(-200);
  
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
  pinMode(end_pin, INPUT);
  stepper.setAcceleration(200);
  stepper.setMaxSpeed(200);
  delay(100);
  initialize();
  stepper.move(2000);
}

void loop() {
  stepper.run();
}
