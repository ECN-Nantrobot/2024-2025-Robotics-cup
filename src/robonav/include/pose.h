#ifndef POSE_H
#define POSE_H

#include "point.h"

namespace ecn
{
struct Pose {
    Point point; // x, y
    float theta; // Orientation in radians

    Pose(float _x = 0, float _y = 0, float _theta = 0) : point(_x, _y), theta(_theta) {}

    // Add a method to access position
    operator Point() const { return point; }

    // Getter methods for x and y
    float x() const { return point.x; }
    float y() const { return point.y; }

    // Setter methods for x and y
    void setX(float _x) { point.x = _x; }
    void setY(float _y) { point.y = _y; }
};

} // namespace ecn

#endif // POSE_H
