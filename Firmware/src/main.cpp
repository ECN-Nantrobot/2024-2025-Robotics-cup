#include <Arduino.h>
#include "config.h"
#include "motorHandler.h"
#include "robot.h"
#include "point.h"

// If you want to use SPIFFS for file loading:
#include "FS.h"
#include "SPIFFS.h"

using namespace ecn;

std::vector<Point> loadedPath;
Robot robot(0, 0, 0, 15, 7, 10, 0.01, 0.5); // x, y, theta, wheelBase, speed, kp, ki, kd

unsigned long lastUpdateTime = 0;
const float dt = 0.05; // 50 ms loop time

bool loadPathFromFile(const char *filename)
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Failed to mount SPIFFS");
    return false;
  }

  File file = SPIFFS.open(filename, "r");
  if (!file || file.isDirectory())
  {
    Serial.println("Failed to open path file");
    return false;
  }

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      continue;

    float x, y;
    int count = sscanf(line.c_str(), "%f %f", &x, &y);
    if (count == 2)
    {
      loadedPath.push_back(Point(x, y));
    }
  }
  file.close();
  return true;
}

void setup() {
  Serial.begin(9600);
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }

  if (SPIFFS.exists("/eb_path_fortest.txt")) {
    Serial.println("File exists!");
    File file = SPIFFS.open("/eb_path_fortest.txt", "r");
    while (file.available()) {
      String line = file.readStringUntil('\n');
      Serial.println(line); // Print each line to verify contents
    }
    file.close();
  } else {
    Serial.println("File not found!");
  }

  loadPathFromFile("/eb_path_fortest.txt");
}


void loop()
{
  unsigned long now = millis();
  if (now - lastUpdateTime >= (unsigned long)(dt * 1000))
  {
    lastUpdateTime = now;

    // Robot internally sets motor speeds.
    robot.followPath(loadedPath, dt);

    // Check if close to end of path
    float distanceToGoal = hypot(loadedPath.back().x - robot.getX(), loadedPath.back().y - robot.getY());
    if (distanceToGoal < 1.0)
    {
      // Stop the robot if close enough to the goal
      setMotorSpeeds(0, 0);
      Serial.println("Goal reached!");
      while (true)
      {
        delay(1000);
      }
    }
  }
}
