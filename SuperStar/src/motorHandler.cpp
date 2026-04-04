#include <math.h>
#include "motorHandler.h"
#include <AccelStepper.h>
#include "Config.h"



bool isBlueTeam() {
  // Check if the selector pin is LOW (blue team)
  return digitalRead(SELECTOR_PIN) == LOW;
}


const double oneStepAngle = 2.0*M_PI / stepsPerRevolution; // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter/2; // [m]
// １m/s = 1 / metersPerStep [steps/s]

// Create motor instances
AccelStepper motorL(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motorR(motorInterfaceType, stepPin2, dirPin2);

long targetPositionL = 0;
long targetPositionR = 0;


void initMotor()
{
  // Set maximum speed and acceleration
  motorL.setMaxSpeed(2000);
  motorL.setAcceleration(500);

  motorR.setMaxSpeed(2000);
  motorR.setAcceleration(500);


  // Invert direction if needed
  motorL.setPinsInverted(true, false, false);

  motorL.setCurrentPosition(0);
  motorR.setCurrentPosition(0);

  motorL.setSpeed(0);
  motorR.setSpeed(0);


  xTaskCreatePinnedToCore(
    allRunSpeed,   // Task function
    "allRunSpeed", // Name
    2048,          // Stack size
    NULL,          // Parameters
    1,             // Lower priority (instead of a high value)
    NULL,          // Task handle
    1              // CPU core (1)
  );

  disableCore1WDT();

  Serial.println("Motors initialized.");

}

void setMotorTarget(float target)
{
  // Convert target from [m/s] to [steps/s]
  target = target / metersPerStep;

  // Set target speeds in steps per second
  motorL.move(target);
  motorR.move(target);
  targetPositionL = motorL.targetPosition();
  targetPositionR = motorR.targetPosition();
}

void setTargetAngle(float angle)
{
  if (!isBlueTeam())
  {
    angle = -angle; // Invert angle for yellow team
  }
  float distance = angle * trackWidth / 2.0; // Convert angle to distance in meters
  distance = distance / metersPerStep; // Convert distance to steps
  Serial.println(distance);
  // Set target positions for the motors
  motorL.move(-distance); // Left motor moves backward
  motorR.move(distance);  // Right motor moves forward
  targetPositionL = motorL.targetPosition();
  targetPositionR = motorR.targetPosition();
}

bool isTargetReached()
{
  // Check if both motors have reached their target positions
  return motorL.distanceToGo() == 0 && motorR.distanceToGo() == 0;
}

void pauseMotors()
{
  // Save the current target positions
  targetPositionL = motorL.targetPosition();
  targetPositionR = motorR.targetPosition();
  // Set high acceleration to stop quickly
  motorL.setAcceleration(5000);
  motorR.setAcceleration(5000);
  // Stop the motors
  motorL.stop();
  motorR.stop();
}

void resumeMotors()
{
  // Reset the acceleration to a normal value
  motorL.setAcceleration(500);
  motorR.setAcceleration(500);
  // Move the motors back to their saved target positions
  motorL.moveTo(targetPositionL);
  motorR.moveTo(targetPositionR);
}


void setMotorSpeeds(float leftSpeed, float rightSpeed)
{
  // speed conversion from [m/s] to [steps/s]
  leftSpeed = leftSpeed / metersPerStep;   
  rightSpeed = rightSpeed / metersPerStep;
  // Serial.println(leftSpeed);
  
  // Set speeds in steps per second
  motorL.setSpeed(leftSpeed);
  motorR.setSpeed(rightSpeed);
}




const float stepDistance = (PI * wheelDiameter) / stepsPerRevolution;

void allRunSpeed(void *pvParameters) {
  while(1) {
    // Check if each motor takes a step
    bool leftStep = motorL.run();
    bool rightStep = motorR.run();

    // If at least one motor stepped, update the odometry:
    if (leftStep || rightStep) {
      // Compute the distance traveled by each wheel.
      // (For simplicity, we assume a true step moves the wheel forward by stepDistance.)
      float dL = leftStep ? stepDistance : 0;
      float dR = rightStep ? stepDistance : 0;
      
      // Differential drive equations:
      // d_center is the average distance traveled by the two wheels.
      float d_center = (dL + dR) / 2.0;
      // dTheta is the change in orientation based on the difference in wheel travel.
      float dTheta = (dR - dL) / trackWidth;
      
      // To update the robot's position accurately, use the robot's orientation
      // at the midpoint of the motion.
      float thetaMid = robotTheta + dTheta / 2.0;
      
      // Protect access to robotX, robotY, and robotTheta with the mutex.
      // if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE) {

        _robotX += d_center * cos(thetaMid);
        _robotY += d_center * sin(thetaMid);
        _robotTheta += dTheta;

      //   xSemaphoreGive(robotXMutex);
      // }
    }
    
    // Optionally add a small delay or yield to let other tasks run.
  }
}