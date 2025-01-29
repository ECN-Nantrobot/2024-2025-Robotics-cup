// #include "Config.h"
// #include <AccelStepper.h>
# include <math.h>

// // Create instances for each motor
// AccelStepper motor1(motorInterfaceType, stepPin1, dirPin1);
// AccelStepper motor2(motorInterfaceType, stepPin2, dirPin2);

// // Given values
const float wheelDiameter = 0.0684;   // meters
// const float trackWidth = 0.185;       // meters
const int stepsPerRevolution = 3200;

const double oneStepAngle = 2.0*M_PI / stepsPerRevolution; // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter/2; // [m]
// １m/s = 1 / metersPerStep [steps/s]

// // Computed values
// const float wheelCircumference = 3.14159 * wheelDiameter; 
// const float stepsPerMeter = stepsPerRevolution / wheelCircumference; 

// // Distances and angles
// const float distance1 = 1;  // meters for the first straight
// const float distance2 = 0.5;  // meters for the second straight
// const float turnAngleDegrees = 90.0;
// const float turnAngleRadians = turnAngleDegrees * 3.14159 / 180.0;

// // Convert distances to steps
// long stepsForDistance1 = (long)(distance1 * stepsPerMeter); // ~3770 steps
// long stepsForDistance2 = (long)(distance2 * stepsPerMeter); // ~1508 steps

// // Steps for turning 90 degrees on the spot
// // Arc length per wheel = (trackWidth / 2) * turnAngleRadians
// float turnArcLength = (trackWidth / 2.0) * turnAngleRadians;
// long stepsForTurn = (long)(turnArcLength * stepsPerMeter);  // ~923 steps




#include "motorHandler.h"
#include <AccelStepper.h>
#include "Config.h"

// Create motor instances
AccelStepper motor1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motor2(motorInterfaceType, stepPin2, dirPin2);

void initMotor()
{
  // Set maximum speed and acceleration
  motor1.setMaxSpeed(20000);
  motor1.setAcceleration(5000);

  motor2.setMaxSpeed(20000);
  motor2.setAcceleration(5000);

  // Invert direction if needed
  motor1.setPinsInverted(true, false, false);

  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);

  Serial.println("Motors initialized.");
}

void setMotorSpeeds(float leftSpeed, float rightSpeed)
{
  // speed conversion from [m/s] to [steps/s]
  leftSpeed = leftSpeed / metersPerStep;   
  rightSpeed = rightSpeed / metersPerStep;
  Serial.println(leftSpeed);
  
  // Set speeds in steps per second
  motor1.setSpeed(leftSpeed);
  motor2.setSpeed(rightSpeed);

  // Run motors at these speeds.
  // Note: runSpeed() needs to be called frequently (e.g. in loop)
  while(1){
    motor1.runSpeed();
    motor2.runSpeed();
  }
  
}
