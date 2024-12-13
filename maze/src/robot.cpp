#include "robot.h"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace ecn
{

Robot::Robot(float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd)
: x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), speed_(speed), leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0)
{
}

float Robot::computePID(float targetAngle, float dt)
{
    float error = targetAngle - theta_;
    while (error > M_PI)
        error -= 2 * M_PI;
    while (error < -M_PI)
        error += 2 * M_PI;

    integral_ += error * dt;
    float derivative = (error - prevError_) / dt;
    prevError_       = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void Robot::updatePosition(float dt)
{
    float v     = (leftSpeed_ + rightSpeed_) / 2.0;        // Average speed
    float omega = (rightSpeed_ - leftSpeed_) / wheelBase_; // Angular velocity

    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += omega * dt;

    while (theta_ > M_PI)
        theta_ -= 2 * M_PI;
    while (theta_ < -M_PI)
        theta_ += 2 * M_PI;
}

void Robot::followPath(const std::vector<Point>& path, float dt)
{
    if (path.empty())
        return;

    // Find the closest point on the path
    float minDist     = std::numeric_limits<float>::max();
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist < minDist) {
            minDist    = dist;
            closestIdx = i;
        }
    }

    // Add a lookahead distance
    const float lookaheadDistance = robot_diameter_ *0.5 +1; // Distance to look ahead on the path (in units)
    size_t targetIdx              = closestIdx;

    for (size_t i = closestIdx + 1; i < path.size(); ++i) {
        float dist = std::hypot(path[i].x - x_, path[i].y - y_);
        if (dist >= lookaheadDistance) {
            targetIdx = i;
            break;
        }
    }

    // If no suitable target point is found, use the last point in the path
    if (targetIdx == closestIdx && closestIdx < path.size() - 1) {
        targetIdx = path.size() - 1;
    }

    // Calculate target orientation
    if (targetIdx < path.size()) {
        float targetX     = path[targetIdx].x;
        float targetY     = path[targetIdx].y;
        float targetAngle = std::atan2(targetY - y_, targetX - x_);

        float controlSignal = computePID(targetAngle, dt);

        // Adjust baseSpeed for smoother movement
        leftSpeed_      = speed_ - controlSignal;
        rightSpeed_     = speed_ + controlSignal;
    }

    // Store the target index for visualization
    targetIdx_ = targetIdx;

    // Update the robot's position
    updatePosition(dt);
}


void Robot::draw(cv::Mat& image, const std::vector<Point>& path, int scale) const
{
    for (size_t i = 0; i < path.size(); ++i) {
        cv::Scalar color = (i == targetIdx_) ? cv::Scalar(0, 0, 255) // Red for the target point
                                               :
                                               cv::Scalar(0, 160, 0); // Green for other points

        cv::circle(image, cv::Point(static_cast<int>(path[i].x * scale), static_cast<int>(path[i].y * scale)), scale * 0.3, color, -1);
    }


    const int robotRadius = scale * robot_diameter_ * 0.5;
    const int wheelRadius = scale * 4;
    const int wheel_width = scale * 2;

    // Heatmap-like color function for wheel speeds: froward: blue, backward: red, no speed: white
    auto getWheelColor = [](float speed) {
        // Normalize speed to a range [-1, 1] for interpolation
        float normalizedSpeed = std::max(-1.0f, std::min(speed / 10.0f, 1.0f)); // Assume max speed is ±10

        int red   = static_cast<int>((1.0f - normalizedSpeed) * 255);
        int blue  = static_cast<int>((1.0f + normalizedSpeed) * 255);
        int green = static_cast<int>((1.0f - std::abs(normalizedSpeed)) * 255);

        return cv::Scalar(blue, green, red);
    };

    // Draw the robot body
    cv::circle(image, cv::Point(static_cast<int>(x_ * scale), static_cast<int>(y_ * scale)), robotRadius, cv::Scalar(100, 100, 100), 1 * scale);

    // Draw the left wheel
    cv::Point leftWheelPos(static_cast<int>((x_ - std::sin(theta_) * wheelBase_ / 2) * scale), static_cast<int>((y_ + std::cos(theta_) * wheelBase_ / 2) * scale));
    cv::RotatedRect leftWheelRect(leftWheelPos, cv::Size2f(wheelRadius * 2, wheel_width), theta_ * 180 / M_PI);
    cv::Point2f vertices[4];
    leftWheelRect.points(vertices);
    std::vector<cv::Point> vertexVec(vertices, vertices + 4);
    cv::fillConvexPoly(image, vertexVec, getWheelColor(leftSpeed_));

    // Draw the right wheel
    cv::Point rightWheelPos(static_cast<int>((x_ + std::sin(theta_) * wheelBase_ / 2) * scale), static_cast<int>((y_ - std::cos(theta_) * wheelBase_ / 2) * scale));
    cv::RotatedRect rightWheelRect(rightWheelPos, cv::Size2f(wheelRadius * 2, wheel_width), theta_ * 180 / M_PI);
    cv::Point2f rightVertices[4];
    rightWheelRect.points(rightVertices);
    std::vector<cv::Point> rightVertexVec(rightVertices, rightVertices + 4);
    cv::fillConvexPoly(image, rightVertexVec, getWheelColor(rightSpeed_));


    // Draw the forward direction
    cv::Point center(static_cast<int>(x_ * scale), static_cast<int>(y_ * scale));
    int arrowLength = robotRadius * 0.5; 
    cv::Point arrowEnd(static_cast<int>(center.x + arrowLength * std::cos(theta_)), static_cast<int>(center.y + arrowLength * std::sin(theta_)));
    double tipLength = 0.3;
    cv::arrowedLine(image, center, arrowEnd, cv::Scalar(0, 0, 0), 2, cv::LINE_AA, 0, tipLength);

    // Draw a cross at the center of the robot aligned with the robot's rotation
    int crossSize = robotRadius * 1.1;
    cv::Point line1Start(center.x + crossSize * std::cos(theta_), center.y + crossSize * std::sin(theta_));
    cv::Point line1End(center.x - crossSize * std::cos(theta_), center.y - crossSize * std::sin(theta_));
    cv::Point line2Start(center.x + crossSize * std::cos(theta_ + M_PI_2), center.y + crossSize * std::sin(theta_ + M_PI_2));
    cv::Point line2End(center.x - crossSize * std::cos(theta_ + M_PI_2), center.y - crossSize * std::sin(theta_ + M_PI_2));
    cv::line(image, line1Start, line1End, cv::Scalar(0, 0, 0), 2);
    cv::line(image, line2Start, line2End, cv::Scalar(0, 0, 0), 2);
}


} // namespace ecn
