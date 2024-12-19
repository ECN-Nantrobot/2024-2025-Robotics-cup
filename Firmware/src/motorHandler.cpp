#include "Config.h"
#include <AccelStepper.h>

// Create instances for each motor
AccelStepper motor1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motor2(motorInterfaceType, stepPin2, dirPin2);

// Given values
const float wheelDiameter = 0.0684;   // meters
const float trackWidth = 0.185;       // meters
const int stepsPerRevolution = 3200;

// Computed values
const float wheelCircumference = 3.14159 * wheelDiameter; 
const float stepsPerMeter = stepsPerRevolution / wheelCircumference; 

// Distances and angles
const float distance1 = 1;  // meters for the first straight
const float distance2 = 0.5;  // meters for the second straight
const float turnAngleDegrees = 90.0;
const float turnAngleRadians = turnAngleDegrees * 3.14159 / 180.0;

// Convert distances to steps
long stepsForDistance1 = (long)(distance1 * stepsPerMeter); // ~3770 steps
long stepsForDistance2 = (long)(distance2 * stepsPerMeter); // ~1508 steps

// Steps for turning 90 degrees on the spot
// Arc length per wheel = (trackWidth / 2) * turnAngleRadians
float turnArcLength = (trackWidth / 2.0) * turnAngleRadians;
long stepsForTurn = (long)(turnArcLength * stepsPerMeter);  // ~923 steps

// State machine for movement
enum MotionState {
  MOVE_STRAIGHT_1,   // Move forward 0.5m
  TURN_90,           // Turn 90°
  MOVE_STRAIGHT_2,   // Move forward 0.2m
  MOVE_STRAIGHT_2_BACK, // Move backward 0.2m
  TURN_MINUS_90,          // Turn -90° (return to original orientation)
  MOVE_STRAIGHT_1_BACK // Move backward 0.5m
};

MotionState currentState = MOVE_STRAIGHT_1;

// Function to print state name
void printState(MotionState state) {
  switch (state) {
    case MOVE_STRAIGHT_1:
      Serial.println("State: MOVE_STRAIGHT_1 (Forward 0.5m)");
      break;
    case TURN_90:
      Serial.println("State: TURN_90 (Turning 90 degrees)");
      break;
    case MOVE_STRAIGHT_2:
      Serial.println("State: MOVE_STRAIGHT_2 (Forward 0.2m)");
      break;
    case MOVE_STRAIGHT_2_BACK:
      Serial.println("State: MOVE_STRAIGHT_2_BACK (Backward 0.2m)");
      break;
    case TURN_MINUS_90:
      Serial.println("State: TURN_MINUS_90 (Turning -90 degrees)");
      break;
    case MOVE_STRAIGHT_1_BACK:
      Serial.println("State: MOVE_STRAIGHT_1_BACK (Backward 0.5m)");
      break;
  }
}

void initMotor() {
    motor1.setMaxSpeed(20000);
    motor1.setAcceleration(5000);

    motor2.setMaxSpeed(20000);
    motor2.setAcceleration(5000);

    // Start the first move
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);

    // Move forward 0.5m
    motor1.moveTo(stepsForDistance1);
    motor2.moveTo(stepsForDistance1);

    motor1.setPinsInverted(true, false, false);
    printState(currentState);
}


void testMotor() {
    // Keep running motors
    motor1.run();
    motor2.run();

    // Check if current movement finished
    if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
        // Transition to next state
        switch (currentState) {
          case MOVE_STRAIGHT_1:
            // Done with forward 0.5m, now turn 90°
            motor1.moveTo(motor1.currentPosition() + stepsForTurn);
            motor2.moveTo(motor2.currentPosition() - stepsForTurn);
            currentState = TURN_90;
            printState(currentState);
            Serial.print(stepsForTurn);
            break;

          case TURN_90:
            // Done turning 90°, now move forward 0.2m
            motor1.moveTo(motor1.currentPosition() + stepsForDistance2);
            motor2.moveTo(motor2.currentPosition() + stepsForDistance2);
            currentState = MOVE_STRAIGHT_2;
            printState(currentState);
            break;

          case MOVE_STRAIGHT_2:
            // Finished moving forward 0.2m
            // Now move backward 0.2m (reverse sequence)
            motor1.moveTo(motor1.currentPosition() - stepsForDistance2);
            motor2.moveTo(motor2.currentPosition() - stepsForDistance2);
            currentState = MOVE_STRAIGHT_2_BACK;
            printState(currentState);
            break;

          case MOVE_STRAIGHT_2_BACK:
            // Finished moving backward 0.2m
            // Now turn -90° to return to original orientation
            motor1.moveTo(motor1.currentPosition() - stepsForTurn);
            motor2.moveTo(motor2.currentPosition() + stepsForTurn);
            currentState = TURN_MINUS_90;
            printState(currentState);
            break;

          case TURN_MINUS_90:
            // Done turning back -90°, now move backward 0.5m to return to start
            motor1.moveTo(motor1.currentPosition() - stepsForDistance1);
            motor2.moveTo(motor2.currentPosition() - stepsForDistance1);
            currentState = MOVE_STRAIGHT_1_BACK;
            printState(currentState);
                        Serial.print(stepsForDistance1);

            break;

          case MOVE_STRAIGHT_1_BACK:
            // Done moving backward 0.5m, cycle complete
            // Repeat cycle: move forward 0.5m again
            motor1.moveTo(motor1.currentPosition() + stepsForDistance1);
            motor2.moveTo(motor2.currentPosition() + stepsForDistance1);
            currentState = MOVE_STRAIGHT_1;
            printState(currentState);
            break;
        }
    }
}
