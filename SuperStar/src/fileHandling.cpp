#include "fileHandling.h"
#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"

using namespace ecn;

bool loadPathFromFile(const char* filename, std::vector<Point>& loadedPath)
{
  // Initialize SPIFFS with formatIfMountFails set to true
  if (!SPIFFS.begin(true))
  {
    Serial.println("Failed to mount SPIFFS");
    return false;
  }

  // Open the file in read mode
  File file = SPIFFS.open(filename, "r");
  if (!file || file.isDirectory())
  {
    Serial.println("Failed to open path file");
    return false;
  }

  // Read the file line by line
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      continue;

    float x, y;
    // Parse the line for two float values
    int count = sscanf(line.c_str(), "%f %f", &x, &y);
    if (count == 2)
    {
      // Append the point to the vector (Point class must support a constructor taking x and y)
      loadedPath.push_back(Point(x, y));
    }
  }
  file.close();
  return true;
}
