#include "robot.h"
#include <cmath>
#include <iostream>
#include "motorHandler.h"

namespace ecn
{

    Robot::Robot(float x, float y, float theta, float target_speed, float kp, float ki, float kd)
        : x_(x), y_(y), theta_(theta), target_speed_(target_speed),
          leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0)
    {
    }

    float Robot::calcAngleError(int target_index)
    {
        float targetX = path_[target_index].x;
        float targetY = path_[target_index].y;
        float target_angle = std::atan2(targetY - y_, targetX - x_);

        float error = target_angle - theta_;
        while (error > M_PI)
            error -= 2 * M_PI;
        while (error < -M_PI)
            error += 2 * M_PI;
        
        Serial.println("target_angle - theta_ = error: " + String(target_angle) + " - " + String(theta_) + " = " + String(error));

        return error;
    }

    float Robot::computePID(float angle_error)
    {
        // float targetAngle = std::atan2(target_y - x_, target_x - y_);
        // float distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

        integral_ += angle_error * dt_;
        float derivative = (angle_error - prevError_) / dt_;
        prevError_ = angle_error;

        float output = kp_ * angle_error + ki_ * integral_ + kd_ * derivative;

        Serial.print("PID output: ");
        Serial.println(output);

        // const float r = 0.0684 / 2; // meters
        // double thetaL = (kp / r) * distance + (kp * L / r) * angle_error;

        const float maxTurnRate = 8.0;
        if (output > maxTurnRate)
            output = maxTurnRate;
        else if (output < -maxTurnRate)
            output = -maxTurnRate;

        return output;
    }

    void Robot::updatePosition()
    {
        // double v = (leftSpeed_ + rightSpeed_) / 2.0f;
        // double omega = (rightSpeed_ - leftSpeed_) / wheelBase_;

        // Euler integration
        //  x_ += v * std::cos(theta_) * dt_;
        //  y_ += v * std::sin(theta_) * dt_;
        //  theta_ += omega * dt_;

        // Runge Kutta 4th order
        float v = (leftSpeed_ + rightSpeed_) / 2.0f;
        float omega = (rightSpeed_ - leftSpeed_) / wheelBase_;

        // RK4 calculations
        float k1_x = v * std::cos(theta_);
        float k1_y = v * std::sin(theta_);
        float k1_theta = omega;

        float k2_x = v * std::cos(theta_ + k1_theta * dt_ / 2.0f);
        float k2_y = v * std::sin(theta_ + k1_theta * dt_ / 2.0f);
        float k2_theta = omega;

        float k3_x = v * std::cos(theta_ + k2_theta * dt_ / 2.0f);
        float k3_y = v * std::sin(theta_ + k2_theta * dt_ / 2.0f);
        float k3_theta = omega;

        float k4_x = v * std::cos(theta_ + k3_theta * dt_);
        float k4_y = v * std::sin(theta_ + k3_theta * dt_);
        float k4_theta = omega;

        x_ += (dt_ / 6.0f) * (k1_x + 2.0f * k2_x + 2.0f * k3_x + k4_x);
        y_ += (dt_ / 6.0f) * (k1_y + 2.0f * k2_y + 2.0f * k3_y + k4_y);
        theta_ += (dt_ / 6.0f) * (k1_theta + 2.0f * k2_theta + 2.0f * k3_theta + k4_theta);

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

        // setMotorSpeeds(leftSpeed_, rightSpeed_);
        // updatePosition();
        return (std::abs(angleError) < 0.02);
    }

    bool Robot::turnToPathOrientation()
    {
        int target_index = findClosestPointOnPath();

        float angle_error = calcAngleError(target_index);

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
            Serial.println("leftSpeed_: " + String(leftSpeed_) + ", rightSpeed_: " + String(rightSpeed_));
        }

        // setMotorSpeeds(leftSpeed_, rightSpeed_);
        // updatePosition();
        return (std::abs(angle_error) < 0.02);
    }

    int Robot::findClosestPointOnPath()
    {
        // Serial.print("eeesppath: ");
        // for (const auto &point : path_)
        // {
        //     Serial.print("(" + String(point.x) + ", " + String(point.y) + ") ");
        // }
        // Serial.println();
        // Find the closest point on the path to the current position
        float minDist = std::numeric_limits<float>::max();
        int closestIdx = 0;
        for (int i = 0; i < path_.size(); ++i)
        {
            float dist = std::hypot(path_[i].x - x_, path_[i].y - y_);
            if (dist < minDist)
            {
                minDist = dist;
                closestIdx = i;
            }
        }

        // Determine the lookahead distance and find the corresponding target point on the path
        const float lookaheadDistance = robot_diameter_ * 0.5f + 1.0f;
        int targetIdx = closestIdx;

        for (int i = closestIdx + 1; i < path_.size(); ++i)
        {
            float dist = std::hypot(path_[i].x - x_, path_[i].y - y_);
            if (dist >= lookaheadDistance)
            {
                targetIdx = i;
                break;
            }
        }

        // Ensure the target index is valid
        if (targetIdx == closestIdx && closestIdx < path_.size() - 1)
            targetIdx = path_.size() - 1;

        return targetIdx;
    }

    void Robot::followPath()
    {
        int target_index = findClosestPointOnPath();

        // Calculate the target angle to the target point
        if (target_index < path_.size())
        {
            float angle_error = calcAngleError(target_index);

            Serial.println("POP:" + String(path_[target_index].x) + ", " + String(path_[target_index].y));

            float controlSignal = computePID(angle_error);

            // Adjust speed based on proximity to obstacles and goal
            float speed_closetogoal = maxSpeed_;
            float distanceToGoal = std::hypot(path_.back().x - x_, path_.back().y - y_);

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

            // If close to the goal: stop
            if (distanceToGoal < 0.02f)
            {
                leftSpeed_ = 0;
                rightSpeed_ = 0;
            }
            // Normal speed
            else
            {
                leftSpeed_ = target_speed_ - controlSignal;
                rightSpeed_ = target_speed_ + controlSignal;

                Serial.println("leftSpeed_: " + String(leftSpeed_) + ", rightSpeed_: " + String(rightSpeed_));
            }

            // setMotorSpeeds(leftSpeed_, rightSpeed_);
        }

        // updatePosition();
    }

} // namespace ecn
