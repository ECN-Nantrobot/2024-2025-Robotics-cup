#include <math.h>
#include "motorHandler.h"
#include <AccelStepper.h>
#include "Config.h"




const double oneStepAngle = 2.0*M_PI / stepsPerRevolution; // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter/2; // [m]
// １m/s = 1 / metersPerStep [steps/s]

// Create motor instances
AccelStepper motorR(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motorL(motorInterfaceType, stepPin2, dirPin2);


void initMotor()
{
  // Set maximum speed and acceleration
  motorL.setMaxSpeed(20000);
  motorL.setAcceleration(5000);

  motorR.setMaxSpeed(20000);
  motorR.setAcceleration(5000);


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
    bool leftStep = motorL.runSpeed();
    bool rightStep = motorR.runSpeed();

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