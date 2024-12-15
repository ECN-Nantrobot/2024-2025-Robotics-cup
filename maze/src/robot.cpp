#include "robot.h"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace ecn
{

Robot::Robot(const Maze& maze, float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd)
: maze_(maze), x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), speed_(speed), leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0) {}

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

    // Calculate PID output
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Clamp the output to a maximum turning rate
    const float maxTurnRate = 8.0; // Limit turning rate
    if (output > maxTurnRate)
        output = maxTurnRate;
    else if (output < -maxTurnRate)
        output = -maxTurnRate;

    return output;
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


bool Robot::turnToGoalOrientation(float dt)
{
    float angleError = targetTheta_ - theta_;

    // Normalize the angle to the range [-PI, PI]
    while (angleError > M_PI)
        angleError -= 2 * M_PI;
    while (angleError < -M_PI)
        angleError += 2 * M_PI;

    if (std::abs(angleError) < 0.02) {
        leftSpeed_  = 0;
        rightSpeed_ = 0;
        return true;
    } else {
        float turnSignal = computePID(targetTheta_, dt);

        leftSpeed_  = -turnSignal ;
        rightSpeed_ = turnSignal;
    }

    updatePosition(dt);

    return false;
}


// float calculateCurvature(const Point& p1, const Point& p2, const Point& p3)
// {
//     // Curvature formula: K = |(x2 - x1)(y3 - y2) - (y2 - y1)(x3 - x2)| / (d12 * d23 * d31)
//     float dx1 = p2.x - p1.x;
//     float dy1 = p2.y - p1.y;
//     float dx2 = p3.x - p2.x;
//     float dy2 = p3.y - p2.y;

//     float cross = std::abs(dx1 * dy2 - dy1 * dx2); // Area determinant
//     float d12   = std::hypot(dx1, dy1);
//     float d23   = std::hypot(dx2, dy2);
//     float d31   = std::hypot(p3.x - p1.x, p3.y - p1.y);

//     if (d12 * d23 * d31 == 0) // Prevent division by zero
//         return 0;

//     return cross / (d12 * d23 * d31);
// }

// float Robot::distanceToClosestObstacle(const Point& p, float searchRadius)
// {
//     // Iterate over a circular region to find the closest obstacle
//     float minDistance = searchRadius;
//     int resolution    = 10; // Number of samples around the robot

//     for (int i = 0; i < resolution; ++i) {
//         float angle     = 2 * M_PI * i / resolution;
//         Point testPoint = { p.x + searchRadius * std::cos(angle), p.y + searchRadius * std::sin(angle) };
//         float distance  = maze_.getDistanceToObstacle(testPoint); // Use Maze's method
//         if (distance < minDistance) {
//             minDistance = distance;
//         }
//     }

//     return minDistance;
// }


void Robot::followPath(const std::vector<Point>& path, const Maze& maze, float dt)
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



        // const float maxCurvatureThreshold = 0.04f; 

        // bool shouldSlowDown = false;

        // // Check curvature
        // if (targetIdx > 0 && targetIdx < path.size() - 1) {
        //     float curvature = calculateCurvature(path[targetIdx - 5], path[targetIdx], path[targetIdx + 5]);
        //     // std::cout << "Curvature: " << curvature << std::endl;
        //     if (curvature > maxCurvatureThreshold) {
        //         shouldSlowDown = true;
        //     }
        // }


        // const float safeDistanceThreshold = 1.0f;

        // // Check obstacles in the forward direction
        // float obstacleDistance = distanceToClosestObstacle(path[targetIdx], robot_diameter_ * 1.5);

        // // Determine if the obstacle is in front of the robot
        // if (obstacleDistance < safeDistanceThreshold) {
        //     // // Calculate the angle to the obstacle
        //     // float obstacleAngle = std::atan2(path[targetIdx].y - y_, path[targetIdx].x - x_);

        //     // // Normalize angles to [-PI, PI]
        //     // float angleDifference = obstacleAngle - theta_;
        //     // while (angleDifference > M_PI)
        //     //     angleDifference -= 2 * M_PI;
        //     // while (angleDifference < -M_PI)
        //     //     angleDifference += 2 * M_PI;

        //     // // Check if the obstacle is within a forward cone (e.g., ±45°)
        //     // const float forwardConeAngle = M_PI / 3.0; // 60 degrees
        //     // if (std::abs(angleDifference) < forwardConeAngle) {
        //         // shouldSlowDown = true;

        //         // Adjust speed based on distance to obstacle
        //         // speed_ = std::max(2.0f, maxSpeed_ * (obstacleDistance / safeDistanceThreshold));
        //     // }
        // }



        float distanceToGoal = std::hypot(path.back().x - x_, path.back().y - y_);

        // Dynamically adjust speed based on distance to the goal
        if (distanceToGoal < 15.0) { // Slow down when near the goal
            speed_ = std::max(0.5f, distanceToGoal / 10.0f * maxSpeed_);

        } else if (isStarting_) {  // Gradual start
            speed_ += 0.2f;        // Increment speed gradually
            if (speed_ >= maxSpeed_) {
                speed_        = maxSpeed_;
                isStarting_   = false; // End the gradual start phase
            }
        } else { // Normal speed
            speed_ = maxSpeed_;
        }


        // if (shouldSlowDown) {
        //     speed_ *= 0.4; 
        // }

        // Stop the robot completely if it's very close to the goal
        if (distanceToGoal < 1.0) {
            leftSpeed_  = 0;
            rightSpeed_ = 0;
        } else {
            leftSpeed_  = speed_ - controlSignal;
            rightSpeed_ = speed_ + controlSignal;
        }

        // float speedDifference      = std::abs(leftSpeed_ - rightSpeed_);
        // float normalizedDifference = speedDifference / (2 * maxSpeed_); // Assuming maxSpeed_ is the max speed of each wheel
        // leftSpeed_ *= (1.0f - normalizedDifference);                    // Reduce speed for sharp turns
        // rightSpeed_ *= (1.0f - normalizedDifference);
        // std::cout << "(1.0 - normalizedDifference): " << (1.0 - normalizedDifference) << std::endl;
    }

    // Store the target index for visualization
    targetIdx_ = targetIdx;

    // Update the robot's position
    updatePosition(dt);
}


void Robot::draw(cv::Mat& image, const std::vector<Point>& path, int scale, const std::vector<Point>& astar_path, const std::vector<Point>& goals) const
{
    // Draw the path
    for (size_t i = 0; i < path.size(); ++i) {
        cv::Scalar color = (i == targetIdx_) ? cv::Scalar(0, 0, 255) // Red for the target point
                                               :
                                               cv::Scalar(0, 160, 0); // Green for other points

        cv::circle(image, cv::Point(static_cast<int>(path[i].x * scale), static_cast<int>(path[i].y * scale)), scale * 0.3, color, -1);
    }


    // draw A* path
    cv::Vec3b colour_astar(255, 50, 50);
    cv::Vec3b colour_astar_points(80, 0, 0);
    const int lineThickness      = 15;
    const int circleThickness    = lineThickness + 2;
    const int rectangleThickness = lineThickness + 2;

    auto drawLine      = [&](const cv::Point& p1, const cv::Point& p2, const cv::Vec3b& color) { cv::line(image, p1, p2, color, lineThickness); };
    auto drawPoint     = [&](const cv::Point& p, const cv::Vec3b& color) { cv::circle(image, p, circleThickness / 2, color, cv::FILLED); };

    for (size_t i = 0; i < astar_path.size() - 1; ++i) {
        cv::Point p1(std::round(astar_path[i].x * scale), std::round(astar_path[i].y * scale));
        cv::Point p2(std::round(astar_path[i + 1].x * scale), std::round(astar_path[i + 1].y * scale));
        drawLine(p1, p2, colour_astar);
        drawPoint(p1, colour_astar_points);
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


    // Draw all goal points with a circle and the number of the goal
    for (size_t i = 0; i < goals.size(); ++i) {
        cv::Point goalPoint(static_cast<int>(goals[i].x * scale), static_cast<int>(goals[i].y * scale));
        cv::circle(image, goalPoint, 2 * scale, cv::Scalar(0, 0, 180), 0.4 * scale);

        // Calculate the size of the text to center it
        std::string text  = std::to_string(i);
        int baseline      = 0;
        cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.07 * scale, 0.3 * scale, &baseline);
        cv::Point textOrg(goalPoint.x - textSize.width / 2 - 2, goalPoint.y + textSize.height / 2 + 2); // Added bias to the left and down

        cv::putText(image, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.1 * scale, cv::Scalar(0, 0, 0), 0.4 * scale);
    }
}


} // namespace ecn
