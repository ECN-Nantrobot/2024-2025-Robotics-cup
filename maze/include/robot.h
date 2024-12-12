#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace ecn
{

class Robot
{
public:
    Robot(float x, float y, float theta, float wheelBase, float kp, float ki, float kd);

    // Update the robot's position using PID control
    void followPath(const std::vector<Point>& path, float dt);

    void draw(cv::Mat& image, const std::vector<Point>& path, int scale) const;

    // Update the robot's position based on wheel speeds
    void updatePosition(float dt);

    float getX() const { return x_; }
    float getY() const { return y_; }
    float getTheta() const { return theta_; }

private:
    float x_;         // x-coordinate of the robot
    float y_;         // y-coordinate of the robot
    float theta_;     // Orientation of the robot in radians
    float wheelBase_; // Distance between the wheels
    float robot_diameter_ = wheelBase_;

    float leftSpeed_;  // Speed of the left wheel
    float rightSpeed_; // Speed of the right wheel

    size_t targetIdx_; // Index of the target point

    // PID control variables
    float kp_, ki_, kd_;
    float prevError_;
    float integral_;

    // Compute the PID control signal
    float computePID(float targetAngle, float dt);
};

} // namespace ecn

#endif // ROBOT_H
