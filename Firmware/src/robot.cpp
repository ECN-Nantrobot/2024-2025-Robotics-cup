#include "robot.h"
#include <cmath>
#include <iostream>
#include "motorHandler.h"

namespace ecn
{

    Robot::Robot(float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd, float dt)
        : x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), speed_(speed),
          leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0), dt_(dt)
    {
    }

    float Robot::calcAngleError(int target_index, const std::vector<Point> &path)
    {
        float targetX = path[target_index].x;
        float targetY = path[target_index].y;
        float target_angle = std::atan2(targetY - y_, targetX - x_);

        float error = target_angle - theta_;
        while (error > M_PI)
            error -= 2 * M_PI;
        while (error < -M_PI)
            error += 2 * M_PI;

        return error;
    }

    float Robot::computePID(float angle_error)
    {
        // Serial.print("Target x: ");
        // Serial.print(target_x);
        // Serial.print(", Target y: ");
        // Serial.println(target_y);
        // Serial.print("Current x: ");
        // Serial.print(x_);
        // Serial.print(", Current y: ");
        // Serial.println(y_);

        // float targetAngle = std::atan2(target_y - x_, target_x - y_);
        // float distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

        integral_ += angle_error * dt_;
        float derivative = (angle_error - prevError_) / dt_;
        prevError_ = angle_error;

        float output = kp_ * angle_error + ki_ * integral_ + kd_ * derivative;

        // Serial.print("Controlsignal: ");
        // Serial.println(output);

        Serial.print("angle_error: ");
        Serial.println(angle_error);

        const float r = 0.0684 / 2; // meters

        // double thetaL = (kp / r) * distance + (kp * L / r) * angle_error;
        // double thetaR = (kp / r) * distance - (kp * L / r) * angle_error;

        const float maxTurnRate = 8.0;
        if (output > maxTurnRate)
            output = maxTurnRate;
        else if (output < -maxTurnRate)
            output = -maxTurnRate;

        return output;
    }

    void Robot::updatePosition()
    {
        float v = (leftSpeed_ + rightSpeed_) / 2.0f;
        float omega = (rightSpeed_ - leftSpeed_) / wheelBase_;

        x_ += v * std::cos(theta_) * dt_;
        y_ += v * std::sin(theta_) * dt_;
        theta_ += omega * dt_;

        while (theta_ > M_PI)
            theta_ -= 2 * M_PI;
        while (theta_ < -M_PI)
            theta_ += 2 * M_PI;
    }

    bool Robot::turnToGoalOrientation()
    {
        float angleError = targetTheta_ - theta_;
        while (angleError > M_PI)
            angleError -= 2 * M_PI;
        while (angleError < -M_PI)
            angleError += 2 * M_PI;

        if (std::abs(angleError) < 0.02)
        {
            leftSpeed_ = 0;
            rightSpeed_ = 0;
        }
        else
        {
            float turnSignal = computePID(targetTheta_);
            leftSpeed_ = -turnSignal;
            rightSpeed_ = turnSignal;
        }

        setMotorSpeeds(leftSpeed_, rightSpeed_);
        updatePosition();
        return (std::abs(angleError) < 0.02);
    }

    bool Robot::turnToPathOrientation(const std::vector<Point> &path)
    {
        int target_index = findClosestPointOnPath(path);

        float angle_error = calcAngleError(target_index, path);

        if (std::abs(angle_error) < 0.02)
        {
            leftSpeed_ = 0;
            rightSpeed_ = 0;
        }
        else
        {
            float turnSignal = computePID(targetTheta_);
            leftSpeed_ = -turnSignal;
            rightSpeed_ = turnSignal;
        }

        setMotorSpeeds(leftSpeed_, rightSpeed_);
        updatePosition();
        return (std::abs(angle_error) < 0.02);
    }

    int Robot::findClosestPointOnPath(const std::vector<Point> &path)
    {
        // Find the closest point on the path to the current position
        float minDist = std::numeric_limits<float>::max();
        int closestIdx = 0;
        for (int i = 0; i < path.size(); ++i)
        {
            float dist = std::hypot(path[i].x - x_, path[i].y - y_);
            if (dist < minDist)
            {
                minDist = dist;
                closestIdx = i;
            }
        }

        // Determine the lookahead distance and find the corresponding target point on the path
        const float lookaheadDistance = robot_diameter_ * 0.5f + 1.0f;
        int targetIdx = closestIdx;

        for (int i = closestIdx + 1; i < path.size(); ++i)
        {
            float dist = std::hypot(path[i].x - x_, path[i].y - y_);
            if (dist >= lookaheadDistance)
            {
                targetIdx = i;
                break;
            }
        }

        Serial.print("Target index: ");
        Serial.println(targetIdx);

        // Ensure the target index is valid
        if (targetIdx == closestIdx && closestIdx < path.size() - 1)
            targetIdx = path.size() - 1;

        return targetIdx;
    }

    void Robot::followPath(const std::vector<Point> &path)
    {
        int target_index = findClosestPointOnPath(path);

        // Calculate the target angle to the target point
        if (target_index < path.size())
        {
            float angle_error = calcAngleError(target_index, path);
            float controlSignal = computePID(angle_error);

            // Adjust speed based on proximity to obstacles and goal
            // float speed_closetobstacle = maxSpeed_;
            float speed_closetogoal = maxSpeed_;
            float distanceToGoal = std::hypot(path.back().x - x_, path.back().y - y_);

            // Slow speed if the robot is close to a goal
            if (distanceToGoal < 15.0f)
            {
                speed_closetogoal = std::max(0.5f, distanceToGoal / 10.0f * maxSpeed_);
            }
            // Slow speed if the robot is starting
            else if (isStarting_)
            {
                speed_closetogoal += 0.2f;
                if (speed_closetogoal >= maxSpeed_)
                {
                    speed_closetogoal = maxSpeed_;
                    isStarting_ = false;
                }
            }
            // Normal speed
            else
            {
                speed_closetogoal = maxSpeed_;
            }

            // speed_ = std::min(speed_closetogoal, speed_closetobstacle);

            // If close to the goal: stop
            if (distanceToGoal < 0.02f)
            {
                leftSpeed_ = 0;
                rightSpeed_ = 0;
            }
            //Normal speed
            else
            {
                leftSpeed_ = speed_ - controlSignal;
                rightSpeed_ = speed_ + controlSignal;
            }

            setMotorSpeeds(leftSpeed_, rightSpeed_);
        }

        updatePosition();
    }

} // namespace ecn
