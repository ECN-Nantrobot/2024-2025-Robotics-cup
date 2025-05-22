#include <math.h>
#include "motorHandler.h"
#include "servoHandler.h"
#include <AccelStepper.h>
#include "Config.h"
#include "debug.h"

SemaphoreHandle_t robotXMutex;

const double oneStepAngle = 2.0 * M_PI / stepsPerRevolution; // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter / 2; // [m]
const float stepDistance = (PI * wheelDiameter) / stepsPerRevolution; // [m]

// Create motor instances
AccelStepper motorR(motorInterfaceType, stepPin1, dirPin1);
AccelStepper motorL(motorInterfaceType, stepPin2, dirPin2);
AccelStepper motorX(motorInterfaceType, stepPinX, dirPinX);

void initMotor() {
    // Set maximum speed and acceleration
    motorL.setMaxSpeed(20000);
    motorL.setAcceleration(2000 / 32);

    motorR.setMaxSpeed(20000);
    motorR.setAcceleration(2000 / 32);

    motorX.setMaxSpeed(20000 / 32);
    motorX.setAcceleration(50000 / 32);

    // Invert direction if needed
    motorL.setPinsInverted(true, false, false);
    motorX.setPinsInverted(true, false, false);

    motorL.setCurrentPosition(0);
    motorR.setCurrentPosition(0);
    motorX.setCurrentPosition(0);

    motorL.setSpeed(0);
    motorR.setSpeed(0);
    motorX.setSpeed(10000 / 32);

    // Create the mutex before starting the task
    robotXMutex = xSemaphoreCreateMutex();
    if (robotXMutex == NULL) {
        SerialCritical("Failed to create robotXMutex!");
    }

    xTaskCreatePinnedToCore(
        allRunSpeed,   // Task function
        "allRunSpeed", // Name
        2048,          // Stack size
        NULL,          // Parameters
        1,             // Priority
        NULL,          // Task handle
        1              // CPU core (1)
    );

    disableCore1WDT();

    SerialLog("Motors initialized.");

    motorL.move(0);
    motorR.move(0);

    initialize_Z_axis();
}

int end_X_sensor = LOW;

void initialize_Z_axis() {
    pinMode(buttonPinX, INPUT_PULLDOWN);

    SerialLog("Initializing X axis...");

    motorX.setCurrentPosition(0);
    moveAxeZ(800, 1);

    setServo(4, 180);

    motorX.move(-9999999); // Move towards the endstop

    // Variables for detecting multiple consecutive HIGH signals
    int consecutiveHighCount = 0;
    const int requiredHighCount = 5; // Number of consecutive HIGH signals needed

    while (1) {
        end_X_sensor = digitalRead(buttonPinX);

        if (end_X_sensor == HIGH) {
            consecutiveHighCount++;
            if (consecutiveHighCount >= requiredHighCount) {
                break; // Exit loop if enough HIGH signals
            }
        } else {
            consecutiveHighCount = 0; // Reset count if LOW is detected
        }

        SerialLog("X Axis Sensor: " + String(end_X_sensor));
        delay(5);
    }

    motorX.setCurrentPosition(0);
    moveAxeZ(300, 1);
    motorX.setCurrentPosition(0);

    SerialSuccess("X Axis initialized.");
}

void moveAxeZ(int position, bool wait) {
  // Move the motor to the target position
  motorX.moveTo(position);  // Assuming motorX controls the Z-axis, change if necessary

  SerialLog("Moving Axe Z to position: " + String(position));

  if (wait) {
      // Wait until the motor reaches the target position
      while (motorX.isRunning()) {
          // Optionally, you can add a small delay to prevent the CPU from being overloaded
          delay(10);
      }
      SerialLog("Axe Z reached position: " + String(position));
  }
  else {
      SerialLog("Axe Z movement started but not waiting for completion.");
  }
}


void setMotorSpeeds(float leftSpeed, float rightSpeed) {
    // Convert speed from [m/s] to [steps/s]
    leftSpeed = leftSpeed / metersPerStep;
    rightSpeed = rightSpeed / metersPerStep;

    // Set speeds in steps per second
    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);

    SerialLog("Motor speeds set: Left Speed = " + String(leftSpeed) + ", Right Speed = " + String(rightSpeed));
}

void go(int distance){
    motorL.setCurrentPosition(0);
    motorR.setCurrentPosition(0);

    motorL.move(distance);
    motorR.move(distance);

      while (motorR.isRunning()) {
          // Optionally, you can add a small delay to prevent the CPU from being overloaded
          delay(10);
      }
}

void allRunSpeed(void *pvParameters) {
    while (1) {
        // Check if each motor takes a step
        bool leftStep = motorL.runSpeed();
        bool rightStep = motorR.runSpeed();
        motorX.run();

        // If at least one motor stepped, update the odometry
        if (leftStep || rightStep) {
            // Compute the distance traveled by each wheel
            float dL = leftStep ? stepDistance : 0;
            float dR = rightStep ? stepDistance : 0;

            // Differential drive equations
            float d_center = (dL + dR) / 2.0;
            float dTheta = (dR - dL) / trackWidth;

            // Update the robot's position accurately
            float thetaMid = robotTheta + dTheta / 2.0;

            // Protect access to robotX, robotY, and robotTheta with the mutex
            // if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE) {

            _robotX += d_center * cos(thetaMid);
            _robotY += d_center * sin(thetaMid);
            _robotTheta += dTheta;

            //   xSemaphoreGive(robotXMutex);
            // }
        }

        // Optionally add a small delay or yield to let other tasks run
    }
}
