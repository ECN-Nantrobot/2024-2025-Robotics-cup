#include <Arduino.h>
#include "config.h"
#include "motorHandler.h"
#include "robot.h"
#include "point.h"
#include "displayHandler.h"
#include "powerSensorHandler.h"

#include "Wire.h"
#include <math.h>
TwoWire myWire(0);
TwoWire wireDisplay(1);

using namespace ecn;

Robot robot(0, 0, 0, 15, 7, 10, 0.01, 0.5, 0.05); // x, y, theta, wheelBase, speed, kp, ki, kd, dt

DisplayHandler display;
std::vector<Point> loadedPath;

volatile float robotX = 0.0;
volatile float robotY = 0.0;
volatile float robotTheta = 0.0;

volatile float _robotX = 0.0;
volatile float _robotY = 0.0;
volatile float _robotTheta = 0.0;

// Pure Pursuit & Control Parameters
const unsigned long dt = 50; // Control loop interval in ms (50ms => 0.05s)
float dt_seconds = dt / 1000.0; // dt in seconds

// // Robot state (in meters, radians)
float currentX = 0.0;
float currentY = 0.0;
float currentTheta = 0.0;

// Timing
unsigned long lastUpdateTime = 0;
size_t lastTargetIndex = 0;

unsigned long startTime = 0;



void checkForResetCommand()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    if (command == "RESET")
    {
      Serial.println("ESP32 restarting...");
      ESP.restart(); // Software Reset
    }
  }
}

const int ledPin = 2; // eingebaute LED auf GPIO 2

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(500);
  
  // Initialize the other modules (motors, display, power sensor)
  initMotor();

  // For testing, we start with a low constant speed; pure pursuit will adjust individual wheel speeds.
  setMotorSpeeds(0, 0);
  display.initDisplay(false, false);
  initPowerSensor();

  pinMode(ledPin, OUTPUT); // Pin als Ausgang setzen

  // Reset timer for control loop
  lastUpdateTime = millis();
  startTime = millis();

  Serial.println("ESP Initialized!");
}


void loop() {
  digitalWrite(ledPin, LOW); // LED aus

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

      if (command == "START") {
        Serial.println("ESP Starting robot...");

        while (true) {
          checkForResetCommand();

          // Run the pure pursuit update every dt (50 ms)
          if (millis() - lastUpdateTime >= dt) {
            lastUpdateTime = millis();

            robot.followPath(loadedPath);

            // Sicherstellen, dass wir die Daten synchron lesen
            if (xSemaphoreTake(robotXMutex, portMAX_DELAY) == pdTRUE)
            {
              currentX = _robotX;
              currentY = _robotY;
              currentTheta = _robotTheta;
              xSemaphoreGive(robotXMutex);

              digitalWrite(ledPin, HIGH); // LED an
              Serial.print("X: ");
              Serial.print(currentX);
              Serial.print(", Y: ");
              Serial.print(currentY);
              Serial.print(", Theta: ");
              Serial.println(currentTheta);
              digitalWrite(ledPin, LOW); // LED aus
            }

            // // Check if close to end of path
            // float distanceToGoal = hypot(loadedPath.back().x - robot.getX(), loadedPath.back().y - robot.getY());
            // if (distanceToGoal < 1.0)
            // {
            //   // Stop the robot if close enough to the goal
            //   setMotorSpeeds(0, 0);
            //   Serial.println("Goal reached!");
            //   while (true)
            //   {
            //     delay(1000);
            //   }
            // }

            display.updatePointsDisplay(robotY);
          }
        }
    }
  }
}






























////////////////////////////
// #include <Arduino.h>
// #include "config.h"
// #include "motorHandler.h"
// #include "robot.h"
// #include "point.h"
// #include "FS.h"
// #include "SPIFFS.h"
// #include <ContinuousStepper.h>

// using namespace ecn;

// std::vector<Point> loadedPath;
// Robot robot(0, 0, 0, 15, 7, 10, 0.01, 0.5, 0.05); // x, y, theta, wheelBase, speed, kp, ki, kd, dt

// unsigned long lastUpdateTime = 0;

// bool loadPathFromFile(const char *filename)
// {
//   File file = SPIFFS.open(filename, "r");
//   if (!file || file.isDirectory())
//   {
//     Serial.println("Failed to open path file");
//     return false;
//   }

//   int lineCount = 0;
//   while (file.available())
//   {
//     String line = file.readStringUntil('\n');
//     line.trim();
//     if (line.length() == 0)
//       continue;

//     float x, y;
//     int count = sscanf(line.c_str(), "%f %f", &x, &y);
//     if (count == 2)
//     {
//       // Convert from cm to m
//       // x /= 100.0;
//       // y /= 100.0;
//       loadedPath.push_back(Point(x, y));

//       // Serial.print("Line ");
//       // Serial.print(lineCount + 1);
//       // Serial.print(": ");
//       // Serial.print(x);
//       // Serial.print(", ");
//       // Serial.println(y);
//       lineCount++;
//     }
//   }
//   file.close();
//   return true;
// }

// void setup()
// {
//   Serial.begin(9600);
//   if (!SPIFFS.begin(true))
//   {
//     Serial.println("Failed to mount SPIFFS");
//     return;
//   }

//   loadPathFromFile("/newtestpath_2.txt");

//   robot.setPosition(loadedPath[0].x, loadedPath[0].y);

//   initMotor();
// }

// void loop()
// {

//   runMotors();
  


//   unsigned long now = millis();
//   if (now - lastUpdateTime >= (unsigned long)(robot.getDt() * 1000))
//   {
//     lastUpdateTime = now;

//     robot.followPath(loadedPath);

//     // Check if close to end of path
//     float distanceToGoal = hypot(loadedPath.back().x - robot.getX(), loadedPath.back().y - robot.getY());
//     if (distanceToGoal < 1.0)
//     {
//       // Stop the robot if close enough to the goal
//       setMotorSpeeds(0, 0);
//       Serial.println("Goal reached!");
//       while (true)
//       {
//         delay(1000);
//       }
//     }
//   }
// }
