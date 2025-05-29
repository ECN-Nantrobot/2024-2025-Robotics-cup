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
        
        // Serial.println("target_angle - theta_ = error: " + String(target_angle) + " - " + String(theta_) + " = " + String(error));

        return error;
    }

    float Robot::computePID(float angle_error)
    {
        integral_ += angle_error * dt_;
        float derivative = (angle_error - prevError_) / dt_;
        prevError_ = angle_error;

        float output = kp_ * angle_error + ki_ * integral_ + kd_ * derivative;


        float pp = kp_ * angle_error;
        float ii = ki_ * integral_;
        float dd = kd_ * derivative;

        // Serial.println("PID output = KP " + String(kp_) + " * " + String(angle_error) + " = " + String(pp) +
        //                " KI " + String(ki_) + " * " + String(integral_) + " = " + String(ii) +
        //                " KD " + String(kd_) + " * " + String(derivative) + " = " + String(dd) + " || = " +
        //                String(output));

        if (output > max_control_output_){
            output = max_control_output_;
            Serial.print("PID output limited to max_control_output_: ");
        }
        else if (output < -max_control_output_){
            output = -max_control_output_;
            Serial.print("PID output limited to -max_control_output_: ");
        }

        return output;
    }

    void Robot::turn(float angle_error)
    {
        float turnsignal = computePID(angle_error);

        // Serial.println("turnsignal: " + String(turnsignal));

        leftSpeed_ = -turnsignal;
        rightSpeed_ = turnsignal;
        
        // Serial.println("start_turning: " + String(start_turning));
        // Serial.println("turnsignal: " + String(turnsignal) + ", turnsignal_limit: " + String(turnsignal_limit));

        if (start_turning == true)
        {
            if (turnsignal > turnsignal_limit || turnsignal < -turnsignal_limit)
            {
                reducing_factor += 1.0 / 15.0;
                leftSpeed_ = -turnsignal * reducing_factor;
                rightSpeed_ = turnsignal * reducing_factor;
                // Serial.println("reducing_factor: " + String(reducing_factor));
                if (reducing_factor > 1)
                {
                    reducing_factor = 0;
                    start_turning = false;
                }
                // Serial.println("Reduced turnsignal: " + String(turnsignal));
            }
        }
        
        if (std::abs(angle_error) < 0.02)
        {
            leftSpeed_ = 0;
            rightSpeed_ = 0;
        }


        // Serial.println("leftSpeed_: " + String(leftSpeed_) + ", rightSpeed_: " + String(rightSpeed_));
    }

    bool Robot::turnToGoalOrientation()
    {
        float angle_error = goals[current_goal_index].theta - theta_;

        while (angle_error > M_PI)
            angle_error -= 2 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2 * M_PI;
        
        // Serial.println("Current Angle (theta_): " + String(theta_) + ", Target Angle (target theta): " + String(goals[current_goal_index].theta) + ", Angle Error: " + String(angle_error));

        turn(angle_error);

        return (std::abs(angle_error) < 0.02);
    }

    bool Robot::turnToPathOrientation()
    {
        int target_index = findClosestPointOnPath();

        float angle_error = calcAngleError(target_index);

        // Serial.println("Current Angle (theta_): " + String(theta_) + ", Angle Error: " + String(angle_error));

        turn(angle_error);

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
        // Serial.println("Number of points on the path: " + String(path_.size() / 2));

        // Check if the path is empty
        if (path_.empty())
        {
            Serial.println("Error: Path is empty.");
            return -1; 
        }

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
        const float lookaheadDistance = 15.0f;
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

        Serial.println("POP:" + String(path_[targetIdx].x) + "," + String(path_[targetIdx].y));

        return targetIdx;
    }

    void Robot::followPath()
    {
        int target_index = findClosestPointOnPath();

        // Calculate the target angle to the target point
        if (target_index < path_.size())
        {
            float angle_error = calcAngleError(target_index);

            float controlSignal = computePID(angle_error);

            // Adjust speed based on proximity to obstacles and goal
            float speed_to_set = target_speed_;

            float distanceToGoal = std::min(
                std::hypot(path_.back().x - x_, path_.back().y - y_),
                std::hypot(goals[current_goal_index].point.x - x_, goals[current_goal_index].point.y - y_)
            );

            // Slow speed if the robot is close to a goal
            if (distanceToGoal < 10.0f)
            {
                speed_to_set = std::max(0.8f, distanceToGoal / 10.0f * target_speed_);
            }
            // Slow speed if the robot is starting
            
            // Serial.print("Control Signal: " + String(controlSignal) + ", ");
            
            if (is_starting_)
            {
                starting_speed_ += 1.0f;
                speed_to_set = starting_speed_;
                // Serial.print("is_starting == true, ");
                // Serial.println("Starting speed: " + String(starting_speed_));

                if (speed_to_set >= target_speed_)
                {
                    speed_to_set = target_speed_;
                    starting_speed_ = 0;
                    is_starting_ = false;
                }
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
                leftSpeed_ = speed_to_set - controlSignal;
                rightSpeed_ = speed_to_set + controlSignal;

                // Serial.println("leftSpeed_: " + String(leftSpeed_) + ", rightSpeed_: " + String(rightSpeed_));
            }

        }

    }

} // namespace ecn
