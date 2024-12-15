#ifndef ROBOT_H
#define ROBOT_H

#include "point.h"
#include "maze.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace ecn
{

class Robot
{
public:
    Robot(const Maze& maze, float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd);

    // Update the robot's position using PID control
    void followPath(const std::vector<Point>& path, const Maze& maze, float dt);

    void draw(cv::Mat& image, const std::vector<Point>& path, int scale, const std::vector<Point>& astar_path, const std::vector<Point>& goals) const;

    // Update the robot's position based on wheel speeds
    void updatePosition(float dt);

    float getX() const { return x_; }
    float getY() const { return y_; }
    float getTheta() const { return theta_; }

    void setSpeed(float speed) { speed_ = speed; }
    void setMaxSpeed(float maxSpeed) { maxSpeed_ = maxSpeed; }
    void setPose(float x, float y, float theta) { x_ = x; y_ = y; theta_ = theta; }

    void setIsStarting(bool isStarting) { isStarting_ = isStarting; }

    float distanceToClosestObstacle(const Point& p, float searchRadius);

    void setTargetTheta(float targetTheta) { targetTheta_ = targetTheta; }
    float getTargetTheta() const { return targetTheta_; }

    bool turnToGoalOrientation(float dt); // Function to turn the robot to the target orientation

    float distanceToGoal(const Point& p) const {
        return std::hypot(p.x - x_, p.y - y_);
    };

private:

    float x_;         // x-coordinate of the robot
    float y_;         // y-coordinate of the robot
    float theta_;     // Orientation of the robot in radians
    float wheelBase_; // Distance between the wheels
    float robot_diameter_ = wheelBase_;

    float leftSpeed_;  // Speed of the left wheel
    float rightSpeed_; // Speed of the right wheel

    float speed_; // Current Speed of the robot
    float maxSpeed_ = 10.0;

    float targetTheta_ = 0.0f; // Target orientation at the goal (default to 0)


    bool isStarting_ = true; // Flag to check if the robot is in the start phase

    const Maze& maze_; // Reference to the maze object


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
