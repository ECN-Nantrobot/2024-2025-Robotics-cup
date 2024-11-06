#include <AccelStepper.h>

// Define pin connections for ESP32
const int dirPin1 = 25;  // Motor 1 direction pin
const int stepPin1 = 26; // Motor 1 step pin
const int dirPin2 = 32;  // Motor 2 direction pin
const int stepPin2 = 33; // Motor 2 step pin

// Define motor interface type
#define motorInterfaceType 1

// Create instances for each motor
AccelStepper motor1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motor2(motorInterfaceType, stepPin2, dirPin2);

void setup() {
    // Set motor parameters for smooth movement for motor 1
    motor1.setMaxSpeed(10000);      // Adjust maximum speed (steps per second)
    motor1.setAcceleration(5000);  // Adjust acceleration (steps per second squared)
    motor1.moveTo(1600*2);           // Set initial target position for motor 1

    // Set motor parameters for smooth movement for motor 2
    motor2.setMaxSpeed(10000);      // Adjust maximum speed (steps per second)
    motor2.setAcceleration(5000);  // Adjust acceleration (steps per second squared)
    motor2.moveTo(3200);           // Set initial target position for motor 2
}

void loop() {
    // Change direction when target position is reached for motor 1
    if (motor1.distanceToGo() == 0) {
        motor1.moveTo(-motor1.currentPosition());
    }

    // Change direction when target position is reached for motor 2
    if (motor2.distanceToGo() == 0) {
        motor2.moveTo(-motor2.currentPosition());
    }

    // Move both motors
    motor1.run();
    motor2.run();
}
