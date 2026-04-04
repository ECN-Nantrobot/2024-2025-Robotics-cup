#include "robot.h"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace ecn
{

Robot::Robot(const Maze& maze, float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd)
: maze_(maze), x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), max_speed_(speed), leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0)
{}

} // namespace ecn
