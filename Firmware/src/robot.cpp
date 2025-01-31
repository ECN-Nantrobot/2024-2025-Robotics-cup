#include "robot.h"
#include <cmath>
#include <iostream>
#include "motorHandler.h" 

namespace ecn
{

Robot::Robot(float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd)
    : x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), speed_(speed),
      leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0)
{
}

float Robot::computePID(float targetAngle, float dt)
{
    float error = targetAngle - theta_;
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    integral_ += error * dt;
    float derivative = (error - prevError_) / dt;
    prevError_ = error;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    const float maxTurnRate = 8.0;
    if (output > maxTurnRate)
        output = maxTurnRate;
    else if (output < -maxTurnRate)
        output = -maxTurnRate;

    return output;
}

void Robot::updatePosition(float dt)
{
    float v = (leftSpeed_ + rightSpeed_) / 2.0f;
    float omega = (rightSpeed_ - leftSpeed_) / wheelBase_;

    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += omega * dt;

    while (theta_ > M_PI) theta_ -= 2 * M_PI;
    while (theta_ < -M_PI) theta_ += 2 * M_PI;
}

bool Robot::turnToGoalOrientation(float dt)
{
    float angleError = targetTheta_ - theta_;
    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

    if (std::abs(angleError) < 0.02) {
        leftSpeed_ = 0;
        rightSpeed_ = 0;
    } else {
        float turnSignal = computePID(targetTheta_, dt);
        leftSpeed_ = -turnSignal;
        rightSpeed_ = turnSignal;
    }

    // Set motor speeds here:
    setMotorSpeeds(leftSpeed_, rightSpeed_);
    updatePosition(dt);
    return (std::abs(angleError) < 0.02);
}

bool Robot::turnToPathOrientation(float dt, const std::vector<Point> &path)
{
    if (path.empty())
        return true;

    float minDist = std::numeric_limits<float>::max();
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }

    const float lookaheadDistance = robot_diameter_ * 0.5f + 1.0f;
    size_t targetIdx = closestIdx;

    for (size_t i = closestIdx + 1; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist >= lookaheadDistance) {
            targetIdx = i;
            break;
        }
    }

    if (targetIdx == closestIdx && closestIdx < path.size() - 1)
        targetIdx = path.size() - 1;

    float targetX = path[targetIdx].x;
    float targetY = path[targetIdx].y;
    float targetAngle = std::atan2(targetY - y_, targetX - x_);

    float angleError = targetTheta_ - theta_;
    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

    if (std::abs(angleError) < 0.02) {
        leftSpeed_ = 0;
        rightSpeed_ = 0;
    } else {
        float turnSignal = computePID(targetTheta_, dt);
        leftSpeed_ = -turnSignal;
        rightSpeed_ = turnSignal;
    }

    // Set motor speeds directly:
    setMotorSpeeds(leftSpeed_, rightSpeed_);
    updatePosition(dt);
    return (std::abs(angleError) < 0.02);
}

void Robot::followPath(const std::vector<Point> &path, float dt)
{
    if (path.empty())
        return;

    float minDist = std::numeric_limits<float>::max();
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }

    const float lookaheadDistance = robot_diameter_ * 0.5f + 1.0f;
    size_t targetIdx = closestIdx;

    for (size_t i = closestIdx + 1; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist >= lookaheadDistance) {
            targetIdx = i;
            break;
        }
    }

    if (targetIdx == closestIdx && closestIdx < path.size() - 1)
        targetIdx = path.size() - 1;

    if (targetIdx < path.size()) {
        float targetX = path[targetIdx].x;
        float targetY = path[targetIdx].y;
        float targetAngle = std::atan2(targetY - y_, targetX - x_);
        float controlSignal = computePID(targetAngle, dt);

        float speed_closetobstacle = maxSpeed_;
        float speed_closetogoal = maxSpeed_;
        float distanceToGoal = std::hypot(path.back().x - x_, path.back().y - y_);

        if (distanceToGoal < 15.0f) {
            speed_closetogoal = std::max(0.5f, distanceToGoal / 10.0f * maxSpeed_);
        } else if (isStarting_) {
            speed_closetogoal += 0.2f;
            if (speed_closetogoal >= maxSpeed_) {
                speed_closetogoal = maxSpeed_;
                isStarting_ = false;
            }
        } else {
            speed_closetogoal = maxSpeed_;
        }

        speed_ = std::min(speed_closetogoal, speed_closetobstacle);

        if (distanceToGoal < 0.02f) {
            leftSpeed_ = 0;
            rightSpeed_ = 0;
        } else {
            leftSpeed_ = speed_ - controlSignal;
            rightSpeed_ = speed_ + controlSignal;
        }

        // Directly command the motors here:
        setMotorSpeeds(leftSpeed_, rightSpeed_);
        updatePosition(dt);
    }
}

} // namespace ecn
