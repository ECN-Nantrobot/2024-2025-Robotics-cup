#include "robot.h"
#include <math.h>
#include "motorHandler.h"
#include <AccelStepper.h>
#include "Config.h"

SemaphoreHandle_t robotXMutex;

// extern Robot robot;

volatile float robotX = 0.0;
volatile float robotY = 0.0;
volatile float robotTheta = 0.0;
volatile float _robotX = 0.0;
volatile float _robotY = 0.0;
volatile float _robotTheta = 0.0;

const double oneStepAngle = 2.0 * M_PI / stepsPerRevolution;   // [rad]
const double metersPerStep = oneStepAngle * wheelDiameter / 2; // [m]
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

    // Create the mutex before starting the task.
    robotXMutex = xSemaphoreCreateMutex();
    if (robotXMutex == NULL)
    {
        Serial.println("Failed to create mutex!");
    }

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

    Serial.println("Motors initialized!");
}

void setMotorSpeeds(float left_speed_to_set, float right_speed_to_set)
{
    // speed conversion from [m/s] to [steps/s]
    double left_speed = left_speed_to_set / metersPerStep;
    double right_speed = right_speed_to_set / metersPerStep;

    // Set speeds in steps per second
    motorL.setSpeed(left_speed);
    motorR.setSpeed(right_speed);
    //   Serial.println("Motor speeds set: " + String(left_speed) + ", " + String(right_speed));
}

const float stepDistance = (PI * wheelDiameter) / stepsPerRevolution;

void allRunSpeed(void *pvParameters)
{
    while (1)
    {
        // Check if each motor takes a step
        bool leftStep = motorL.runSpeed();
        bool rightStep = motorR.runSpeed();

        // If at least one motor stepped, update the odometry:
        if (leftStep || rightStep)
        {
            // Compute the distance traveled by each wheel.
            // (For simplicity, we assume a true step moves the wheel forward by stepDistance.)
            float dL = leftStep ? (motorL.speed() > 0 ? stepDistance : -stepDistance) : 0;
            float dR = rightStep ? (motorR.speed() > 0 ? stepDistance : -stepDistance) : 0;

            // Differential drive equations:
            // d_center is the average distance traveled by the two wheels.
            float d_center = (dL + dR) / 2.0;
            // dTheta is the change in orientation based on the difference in wheel travel.
            float dTheta = (dR - dL) / trackWidth;

            // To update the robot's position accurately, use the robot's orientation at the midpoint of the motion.
            // float thetaMid = _robotTheta + dTheta;

            if (fabs(dTheta) < 1e-6)
            { // Falls Drehung sehr klein -> Geradeausbewegung
                _robotX = _robotX + d_center * cos(_robotTheta + dTheta);
                _robotY = _robotY + d_center * sin(_robotTheta + dTheta);
                // Serial.println("Geradeausbewegung");
            }
            else
            { // Kreisbogenbewegung
                double R = d_center / dTheta;
                _robotX = _robotX + R * (sin(_robotTheta + dTheta) - sin(_robotTheta));
                _robotY = _robotY - R * (cos(_robotTheta + dTheta) - cos(_robotTheta));
            }

            _robotTheta += dTheta;

            if (_robotTheta > M_PI)
                _robotTheta -= 2 * M_PI;
            else if (_robotTheta < -M_PI)
                _robotTheta += 2 * M_PI;

            // Protect access to robotX, robotY, and robotTheta with the mutex.
            // if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE) {

            // _robotX += d_center * cos(thetaMid) * 100; // in cm
            // _robotY += d_center * sin(thetaMid) * 100; // in cm

            // Serial.print("X: ");
            // Serial.print(_robotX);
            // Serial.print(" Y: ");
            // Serial.print(_robotY);
            // Serial.print(" Theta: ");
            // Serial.println(_robotTheta);

            //   xSemaphoreGive(robotXMutex);
            // }
        }

        // Optionally add a small delay or yield to let other tasks run.
    }
}