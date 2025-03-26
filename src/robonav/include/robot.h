#ifndef ROBOT_H
#define ROBOT_H

#include "maze.h"
#include "point.h"
#include "pose.h"
#include "position.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace ecn
{

struct ObstaclePoint {
    Point position; // Position of the point
    bool isFree;    // True if the point is free, false if blocked
};
class Robot
{
public:
    Robot(const Maze& maze, float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd);

    // Update the robot's position using PID control
    void followPath(const std::vector<Point>& path, const Maze& maze, float dt);

    void draw(cv::Mat& image,
              const std::vector<Point>& path,
              int scale,
              const std::vector<Point>& astar_path,
              const std::vector<Point>& goals,
              const std::vector<Point>& eb_path,
              const std::vector<Position>& astar_path_new) const;

    // Update the robot's position based on wheel speeds
    void updatePosition(float dt);

    float getX() const { return x_; }
    float getY() const { return y_; }
    Point getPosition() const { return Point{x_, y_}; }
    float getTheta() const { return theta_; }

    void setSpeed(float speed) { speed_ = speed; }
    void setMaxSpeed(float maxSpeed) { max_speed_ = maxSpeed; }
    void setPose(float x, float y, float theta)
    {
        x_     = x;
        y_     = y;
        theta_ = theta;
    }

    void setIsStarting(bool isStarting) { isStarting_ = isStarting; }

    void setTargetTheta(float targetTheta) { targetTheta_ = targetTheta * M_PI / 180.0f; }
    float getTargetTheta() const { return targetTheta_; }

    bool turnToGoalOrientation(float dt);
    bool turnToPathOrientation(float dt, const std::vector<Point>& path);

    float distanceToClosestObstacleInFront(float searchRadius, float coneAngle);


    float distanceToGoal(const Point& p) const { return std::hypot(p.x - x_, p.y - y_); };


    // For Ros
    float getLeftSpeed() const { return leftSpeed_; }
    float getRightSpeed() const { return rightSpeed_; }
    float getLinearVelocity() const { return (leftSpeed_ + rightSpeed_) / 2.0; }
    float getAngularVelocity() const { return (rightSpeed_ - leftSpeed_) / wheelBase_; }

    std::vector<Pose> goals;
    int goal_index = 1;


private:
    float x_;         // x-coordinate of the robot
    float y_;         // y-coordinate of the robot
    float theta_;     // Orientation of the robot in radians
    float wheelBase_; // Distance between the wheels
    float robot_diameter_ = wheelBase_;

    

    std::vector<ObstaclePoint> forwardConePoints_; // Member variable to store the points
    void checkForwardObstacles(float searchRadius, float coneAngle, int resolution_radial, int resolution_angle);


    float leftSpeed_;  // Speed of the left wheel
    float rightSpeed_; // Speed of the right wheel

    float speed_; // Current Speed of the robot
    float max_speed_;

    float targetTheta_ = 0.0f; // Target orientation at the goal (default to 0)

    const float sensor_zero_offset_ = robot_diameter_ / 2 + 2;


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
