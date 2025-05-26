#include <Arduino.h>
#include "config.h"
#include "motorHandler.h"
#include "robot.h"
#include "point.h"
#include "displayHandler.h"
#include "powerSensorHandler.h"
#include "fileHandling.h"  // Our file-handling module
#include "Ultrasonic.h"

#include "Wire.h"
#include <math.h>
TwoWire myWire(0);
TwoWire wireDisplay(1);

using namespace ecn;

DisplayHandler display;
std::vector<Point> loadedPath;
capteur ultrasonic(8, 9);

volatile float robotX = 0.0;
volatile float robotY = 0.0;
volatile float robotTheta = 0.0;

volatile float _robotX = 0.0;
volatile float _robotY = 0.0;
volatile float _robotTheta = 0.0;

// Pure Pursuit & Control Parameters
const float v = 0.1;              // Robot forward speed (m/s)
const float lookAheadDistance = 0.05; // Look-ahead distance (m)
const unsigned long controlInterval = 50; // Control loop interval in ms (50ms => 0.05s)
float dt_seconds = controlInterval / 1000.0; // dt in seconds

// // Robot state (in meters, radians)
// float robotX = 0.0;
// float robotY = 0.0;
// float robotTheta = 0.0;

// Timing
unsigned long lastUpdateTime = 0;
size_t lastTargetIndex = 0;

unsigned long startTime = 0;

// Define PI if not already defined.
#ifndef PI
  #define PI 3.14159265358979323846
#endif

/**
 * @brief Normalizes an angle to the interval [-PI, PI].
 */
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2*PI;
  while (angle < -PI) angle += 2*PI;
  return angle;
}

/**
 * @brief Performs one pure pursuit control update.
 *
 * Finds a target point on the path (converted from cm to m) that is at least
 * lookAheadDistance away from the robot's current position, computes the curvature,
 * then calculates the differential wheel speeds and sends them to the motors.
 * For simulation/testing, the robot's state is also updated.
 */
void purePursuitUpdate() {
  // 1. Find the target point on the path (convert from cm to m)
  bool foundTarget = false;
  float targetX = 0.0, targetY = 0.0;
  
  // Start search from lastTargetIndex instead of 0.
  for (size_t i = lastTargetIndex; i < loadedPath.size(); i++) {
    // Convert loaded path point from cm to m
    float px = loadedPath[i].x / 100.0;
    float py = loadedPath[i].y / 100.0;
    float dist = sqrt(pow(px - robotX, 2) + pow(py - robotY, 2));
    if (dist >= lookAheadDistance) {
      targetX = px;
      targetY = py;
      foundTarget = true;
      lastTargetIndex = i;  // Update global index so future searches start here.
      break;
    }
  }
  // If no target found, use the final point of the path.
  if (!foundTarget) {
    targetX = loadedPath.back().x / 100.0;
    targetY = loadedPath.back().y / 100.0;
    lastTargetIndex = loadedPath.size() - 1;
  }
  
  // 2. Compute the angle to the target and the relative angle (alpha)
  float angleToTarget = atan2(targetY - robotY, targetX - robotX);
  float alpha = normalizeAngle(angleToTarget - robotTheta);
  
  // 3. Compute curvature k = 2*sin(alpha)/L_d and the resulting angular velocity
  float k = 2.0 * sin(alpha) / lookAheadDistance;
  float omega = k * v;  // Angular velocity (rad/s)
  
  // 4. Convert the desired angular velocity to differential wheel speeds:
  //    v_left = v - (trackWidth/2)*omega,  v_right = v + (trackWidth/2)*omega
  float leftSpeed = v - (trackWidth / 2.0) * omega;
  float rightSpeed = v + (trackWidth / 2.0) * omega;
  
  // 5. Send motor speed commands (in m/s)
  setMotorSpeeds(leftSpeed, rightSpeed);

  display.updatePointsDisplay(lastTargetIndex);

  // For debugging: print the current state and target info.
  Serial.print("leftSpeed ");
  Serial.print(leftSpeed);
  Serial.print(" rightSpeed ");
  Serial.print(rightSpeed);
  Serial.print(" Robot: x=");
  Serial.print(robotX, 3);
  Serial.print(" y=");
  Serial.print(robotY, 3);
  Serial.print(" theta=");
  Serial.print(robotTheta, 3);
  Serial.print(" | Target: x=");
  Serial.print(targetX, 3);
  Serial.print(" y=");
  Serial.print(targetY, 3);
  Serial.print(" | alpha=");
  Serial.print(alpha, 3);
  Serial.print(" | ω=");
  Serial.println(omega, 3);
  
  // // 6. Update robot state (simulation)
  // //    In a real robot, state would be updated by sensor feedback.
  // robotX += v * cos(robotTheta) * dt_seconds;
  // robotY += v * sin(robotTheta) * dt_seconds;
  // robotTheta = normalizeAngle(robotTheta + omega * dt_seconds);

  // if (xSemaphoreTakeRecursive(robotXMutex, portMAX_DELAY) == pdTRUE) {
    // update variables...
    robotX = _robotX;
    robotY = _robotY;
    robotTheta = _robotTheta;
    // xSemaphoreGiveRecursive(robotXMutex);
  // }
  
  // 7. Check if we are close to the final point (goal reached)
  float finalX = loadedPath.back().x / 100.0;
  float finalY = loadedPath.back().y / 100.0;
  float error = sqrt(pow(finalX - robotX, 2) + pow(finalY - robotY, 2));
  if (error < 0.05) {  // within 5 cm of goal
    setMotorSpeeds(0, 0);
    Serial.println("Goal reached!");
    while (1) { delay(1000); }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  delay(500);


  // Load path file from SPIFFS
  if (loadPathFromFile("/sinus_path.txt", loadedPath)) {
    Serial.println("Loaded path points:");
  } else {
    Serial.println("Failed to load path file.");
  }
  
  // Initialize robot state from the first two points (convert from cm to m)
  if (loadedPath.size() >= 2) {
    robotX = loadedPath[0].x / 100.0;
    robotY = loadedPath[0].y / 100.0;
    float secondX = loadedPath[1].x / 100.0;
    float secondY = loadedPath[1].y / 100.0;
    robotTheta = atan2(secondY - robotY, secondX - robotX);
    _robotX = robotX;
    _robotY = robotY;
    _robotTheta = robotTheta;
  } else {
    Serial.println("Path too short to initialize robot state.");
  }
  
  // Initialize the other modules (motors, display, power sensor)
  initMotor();

  // For testing, we start with a low constant speed; pure pursuit will adjust individual wheel speeds.
  setMotorSpeeds(0, 0);
  display.initDisplay(false, false);
  initPowerSensor();
  
  // Reset timer for control loop
  lastUpdateTime = millis();
  startTime = millis();
}


float position = 1;
void loop() {
     // Run the pure pursuit update every controlInterval (50 ms)
    if (millis() - lastUpdateTime >= controlInterval) {
      lastUpdateTime = millis();
      if (ultrasonic.distance()<lookAheadDistance)
      {
        Serial.println("Obstacle detected, stopping motors.");
        setMotorSpeeds(0,0);
        return;
      }
      purePursuitUpdate();
      display.updatePointsDisplay(robotY);
    } 
}
