#ifndef FILE_HANDLING_H
#define FILE_HANDLING_H

#include <vector>
#include "point.h"  // Make sure this header defines your Point class

using namespace ecn;

/**
 * @brief Loads a path from a file stored in SPIFFS.
 *
 * The file is expected to contain one coordinate pair per line (formatted as "x y").
 *
 * @param filename The name (including path) of the file to read.
 * @param loadedPath Reference to a vector of Points that will be filled with the path.
 * @return true if the file was successfully loaded, false otherwise.
 */
bool loadPathFromFile(const char* filename, std::vector<Point>& loadedPath);

#endif
