#include "Config.h"
# include <math.h>
#include "motorHandler.h"
#include <ContinuousStepper.h>

const float wheelDiameter = 0.0684;   // meters
const int stepsPerRevolution = 3200;

const double oneStepAngle = 2.0*M_PI / stepsPerRevolution; // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter/2 *100; // [m] //[cm]
// const float wheelCircumference = 3.14159 * wheelDiameter; 
// const float stepsPerMeter = stepsPerRevolution / wheelCircumference; 

ContinuousStepper<StepperDriver> motor_left;
ContinuousStepper<StepperDriver> motor_right;

void initMotor()
{
  motor_left.begin(stepPin1, dirPin1);
  motor_right.begin(stepPin2, dirPin2);

  Serial.println("Motors initialized.");
}

void setMotorSpeeds(float leftSpeed, float rightSpeed)
{
  // speed conversion from [m/s] to [steps/s]
  leftSpeed /= - metersPerStep;
  rightSpeed /= metersPerStep;

  motor_left.spin(leftSpeed);
  motor_right.spin(rightSpeed);
  // Serial.print("motor left speed: ");
  // Serial.println(leftSpeed);
  // Serial.print("motor right speed: ");
  // Serial.println(rightSpeed);
}

void runMotors()
{
  motor_left.loop();
  motor_right.loop();

}
