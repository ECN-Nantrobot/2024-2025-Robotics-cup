#include "robot.h"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace ecn
{

Robot::Robot(const Maze& maze, float x, float y, float theta, float wheelBase, float speed, float kp, float ki, float kd)
: maze_(maze), x_(x), y_(y), theta_(theta), wheelBase_(wheelBase), speed_(speed), leftSpeed_(0), rightSpeed_(0), kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0)
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

        leftSpeed_  = -turnSignal;
        rightSpeed_ = turnSignal;
    }

    updatePosition(dt);

    return false;
}

bool Robot::turnToPathOrientation(float dt, const std::vector<Point>& path)
{

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
    const float lookaheadDistance = robot_diameter_ * 0.5 + 1; // Distance to look ahead on the path (in units)
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
    float targetX     = path[targetIdx].x;
    float targetY     = path[targetIdx].y;
    float targetAngle = std::atan2(targetY - y_, targetX - x_);

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

        leftSpeed_  = -turnSignal;
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


// float Robot::distanceToClosestObstacleInFront(float searchRadius, float coneAngle)
// {
//     const float safeDistanceThreshold = robot_diameter_ * 2; // Safe distance to slow down

//     float minDistance     = searchRadius;   // Initialize with the search radius
//     int resolution        = coneAngle / 10; // Number of rays (higher is more precise but slower)
//     float resolution_rad  = resolution * M_PI / 180.0f;
//     float coneangle_rad   = coneAngle * M_PI / 180.0f;
//     int resolution_radial = 5; // Number of radial rays

//     int minDistanceCount = 0;
//     int totalPoints      = (resolution + 1) * (resolution_radial + 1);

//     for (int i = 0; i <= resolution; ++i) {
//         // Compute the angle of the current ray within the forward cone
//         float angle = theta_ - (coneangle_rad / 2) + (coneangle_rad * (static_cast<float>(i) / resolution));

//         // Compute the test point in the forward direction
//         for (int j = 0; j <= resolution_radial; ++j) {
//             float radialDistance = robot_diameter_ / 2 + searchRadius * (static_cast<float>(j) / resolution_radial);
//             Point testPoint      = { x_ + radialDistance * std::cos(angle), y_ + radialDistance * std::sin(angle) };
//             if (!maze_.isFree(testPoint)) {
//                 minDistance = std::min(minDistance, radialDistance - robot_diameter_ / 2);
//                 if (minDistance == radialDistance - robot_diameter_ / 2) {
//                     minDistanceCount++;
//                 }
//             }
//         }
//     }

//     // Calculate the slow down factor based on the distance and the number of points that are not free
//     float slowDownFactor = 1.0f;
//     if (minDistance < safeDistanceThreshold) {
//         float obstacleDensity = static_cast<float>(minDistanceCount) / totalPoints;
//         slowDownFactor        = std::min(1.0f, obstacleDensity * (safeDistanceThreshold / minDistance));
//     }

//     return slowDownFactor;
// }

void Robot::checkForwardObstacles(float searchRadius, float coneAngle, int resolution_radial, int resolution_angle)
{
    forwardConePoints_.clear(); // Clear previous points
    float coneangle_rad = coneAngle * M_PI / 180.0f;

    for (int i = 0; i <= resolution_angle; ++i) {
        float angle = theta_ - (coneangle_rad / 2) + (coneangle_rad * static_cast<float>(i) / resolution_angle);

        for (int j = 0; j <= resolution_radial; ++j) {
            float radialDistance = sensor_zero_offset_ + searchRadius * (static_cast<float>(j) / resolution_radial);

            Point testPoint = { x_ + radialDistance * std::cos(angle), y_ + radialDistance * std::sin(angle) };

            bool free = maze_.isFreeNotPermanent(testPoint);
            forwardConePoints_.push_back({ testPoint, free });
        }
    }
}


float Robot::distanceToClosestObstacleInFront(float searchRadius, float coneAngle)
{
    // Perform the obstacle check and store points
    checkForwardObstacles(searchRadius, coneAngle, 6, coneAngle / 10);

    float minDistance = searchRadius; // Start with max search radius
    int blockedCount  = 0;            // Number of blocked points
    int totalPoints   = forwardConePoints_.size();

    // Loop through the points to compute the minimum distance and count blocked points
    for (const auto& point : forwardConePoints_) {
        if (!point.isFree) {
            float rawDistance      = std::hypot(point.position.x - x_, point.position.y - y_);
            float distance = std::max(0.0f, rawDistance - sensor_zero_offset_);
            minDistance = std::min(minDistance, distance);
            ++blockedCount;
        }
    }

    if (blockedCount == 0) {
        return 1.0f; // Full speed
    }

    float obstacleDensity = std::min(1.0f, static_cast<float>(blockedCount) / totalPoints*0.7f);

    const float safeDistanceThreshold = robot_diameter_ * 2;

    // Compute slowdown factor based on proximity and obstacle density
    float weight_distance = 0.85f;
    float slowdownFactor  = std::min(1.0f, (1 - weight_distance) * (1 - obstacleDensity) + weight_distance * (minDistance / safeDistanceThreshold));
    // std ::cout << "1 - obstacleDensity: " << (1 - obstacleDensity) << ", Mindistance/thresh: " << minDistance / safeDistanceThreshold << ", Slowdown factor: " << slowdownFactor << std::endl;

    return slowdownFactor;
}


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
    const float lookaheadDistance = robot_diameter_ * 0.5 + 1; // Distance to look ahead on the path (in units)
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

        // // Check curvature
        // if (targetIdx > 0 && targetIdx < path.size() - 1) {
        //     float curvature = calculateCurvature(path[targetIdx - 5], path[targetIdx], path[targetIdx + 5]);
        //     // std::cout << "Curvature: " << curvature << std::endl;
        //     if (curvature > maxCurvatureThreshold) {
        //         shouldSlowDown = true;
        //     }
        // }

        float speed_closetobstacle     = maxSpeed_;
        float slowdown_factor_obstacle = distanceToClosestObstacleInFront(robot_diameter_ * 2, 40.0f);

        if (slowdown_factor_obstacle < 1.0f) {
            speed_closetobstacle = maxSpeed_ * slowdown_factor_obstacle; // Adjust speed based on slowdown factor
            // std::cout << "Obstacle detected! Slowdown factor: " << slowdown_factor_obstacle << ", Speed: " << speed_ << std::endl;
        } else {
            speed_closetobstacle = maxSpeed_;
        }


        float speed_closetogoal = maxSpeed_;
        float distanceToGoal = std::hypot(path.back().x - x_, path.back().y - y_);

        // Dynamically adjust speed based on distance to the goal
        if (distanceToGoal < 15.0) { // Slow down when near the goal
            speed_closetogoal = std::max(0.5f, distanceToGoal / 10.0f * maxSpeed_);

        } else if (isStarting_) { // Gradual start
            speed_closetogoal += 0.2f; // Increment speed gradually
            if (speed_closetogoal >= maxSpeed_) {
                speed_closetogoal = maxSpeed_;
                isStarting_ = false; // End the gradual start phase
            }
        } else { // Normal speed
            speed_closetogoal = maxSpeed_;
        }

        speed_ = std::min(speed_closetogoal, speed_closetobstacle);


        // Stop the robot completely if it's very close to the goal
        if (distanceToGoal < 0.02) {
            leftSpeed_  = 0;
            rightSpeed_ = 0;
        } else {
            leftSpeed_  = speed_ - controlSignal;
            rightSpeed_ = speed_ + controlSignal;
        }

        // float speedDifferenc10e      = std::abs(leftSpeed_ - rightSpeed_);
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


void Robot::draw(cv::Mat& image, const std::vector<Point>& path, int scale, const std::vector<Point>& astar_path, const std::vector<Point>& goals, const std::vector<Point>& eb_path) const
{
    // Create a transparent overlay
    cv::Mat overlay = image.clone();

    // Draw Ealstic Band path
    for (const auto& point : eb_path) {
        cv::rectangle(overlay, cv::Point(point.x * scale, point.y * scale), cv::Point((point.x + 1) * scale, (point.y + 1) * scale),
                      cv::Scalar(point.colour[0], point.colour[1], point.colour[2], 255), cv::FILLED);
    }

    // Draw Circles aroung the points
    for (const auto& point : eb_path) {
        cv::circle(overlay, cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2),
                   point.radius * scale, // Adjust radius based on scale
                   cv::Scalar(point.colour[0], point.colour[1], point.colour[2], 255),
                   scale / 5); // Thickness proportional to scale
    }

    // Blend the overlay with the simulation
    float alpha_blend = 0.14; // (0.0 = fully transparent, 1.0 = fully opaque)
    cv::addWeighted(overlay, alpha_blend, image, 1.0 - alpha_blend, 0.0, image);


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

    // draw A* path
    cv::Vec3b colour_astar(255, 50, 50);
    cv::Vec3b colour_astar_points(80, 0, 0);
    const int lineThickness      = 8;
    const int circleThickness    = lineThickness + 2;
    const int rectangleThickness = lineThickness + 2;

    auto drawLine  = [&](const cv::Point& p1, const cv::Point& p2, const cv::Vec3b& color) { cv::line(image, p1, p2, color, lineThickness); };
    auto drawPoint = [&](const cv::Point& p, const cv::Vec3b& color) { cv::circle(image, p, circleThickness / 2, color, cv::FILLED); };

    for (size_t i = 0; i < astar_path.size() - 1; ++i) {
        cv::Point p1(std::round(astar_path[i].x * scale), std::round(astar_path[i].y * scale));
        cv::Point p2(std::round(astar_path[i + 1].x * scale), std::round(astar_path[i + 1].y * scale));
        drawLine(p1, p2, colour_astar);
        drawPoint(p1, colour_astar_points);
    }


    // Draw the smoothed Elastic Band path
    for (size_t i = 0; i < path.size(); ++i) {
        cv::Scalar color = (i == targetIdx_) ? cv::Scalar(0, 255, 0) // Red for the target point
                                               :
                                               cv::Scalar(0, 0, 160); // Green for other points

        cv::circle(image, cv::Point(static_cast<int>(path[i].x * scale), static_cast<int>(path[i].y * scale)), scale * 0.5, color, -1);
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
    cv::Point center(static_cast<int>(x_ * scale), static_cast<int>(y_ * scale));
    cv::circle(image, center, robotRadius, cv::Scalar(100, 100, 100), 1 * scale);

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



    // float coneangle_rad = 40 * M_PI / 180.0f;
    // int resolution      = 40 / 10; // Number of rays for sampling

    // float theta_temp = theta_;

    // int searchRadius = robot_diameter_ * 2;
    // Point closestPoint; // To store the closest obstacle point
    // float minDistance     = searchRadius;
    // int resolution_radial = 10; // Number of radial rays

    // for (int i = 0; i <= resolution; ++i) {
    //     // Compute the angle for this ray within the cone
    //     float angle = theta_ - (coneangle_rad / 2) + (coneangle_rad * (static_cast<float>(i) / resolution));

    //     // Compute the test point in the forward direction
    //     for (int j = 0; j <= resolution_radial; ++j) {
    //         float radialDistance = robot_diameter_ / 2 + searchRadius * (static_cast<float>(j) / resolution_radial);
    //         Point testPoint      = { x_ + radialDistance * std::cos(angle), y_ + radialDistance * std::sin(angle) };

    //         cv::Point testPointScaled(static_cast<int>(testPoint.x * scale), static_cast<int>(testPoint.y * scale));
    //         cv::circle(image, testPointScaled, 5, cv::Scalar(255, 255, 255), -1); // Red for the closest obstacle

    //         if (!maze_.isFree(testPoint)) {
    //             minDistance = std::min(minDistance, radialDistance - robot_diameter_ / 2);
    //             cv::circle(image, testPointScaled, 5, cv::Scalar(0, 0, 255), -1); // Red for the closest obstacle
    //             if (minDistance < robot_diameter_ * 2) {
    //                 cv::circle(image, testPointScaled, 5, cv::Scalar(0, 255, 0), -1); // Red for the closest obstacle
    //             }
    //         }
    //     }
    // }

    // // draw the forward cone boundary
    // cv::Point leftConeEdge(static_cast<int>((x_ + searchRadius * std::cos(theta_ - coneangle_rad / 2)) * scale),
    //                        static_cast<int>((y_ + searchRadius * std::sin(theta_ - coneangle_rad / 2)) * scale));
    // cv::Point rightConeEdge(static_cast<int>((x_ + searchRadius * std::cos(theta_ + coneangle_rad / 2)) * scale),
    //                         static_cast<int>((y_ + searchRadius * std::sin(theta_ + coneangle_rad / 2)) * scale));

    // cv::line(image, center, leftConeEdge, cv::Scalar(0, 255, 0), 1.5); 
    // cv::line(image, center, rightConeEdge, cv::Scalar(0, 255, 0), 1.5);


    // Draw forward cone points using stored data
    for (const auto& point : forwardConePoints_) {
        cv::Point testPointScaled(static_cast<int>(point.position.x * scale), static_cast<int>(point.position.y * scale));
        cv::Scalar color = point.isFree ? cv::Scalar(240, 240, 240) : cv::Scalar(0, 0, 255); // White for free, Red for blocked
        cv::circle(image, testPointScaled, 0.3* scale, color, -1);
    }

    // Draw cone boundaries
    float coneangle_rad = 40 * M_PI / 180.0f;
    float searchRadius  = robot_diameter_ * 2;
    cv::Point leftConeEdge(static_cast<int>(std::round((x_ + (sensor_zero_offset_ + searchRadius) * std::cos(theta_ - coneangle_rad / 2)) * scale)),
                           static_cast<int>(std::round((y_ + (sensor_zero_offset_ + searchRadius) * std::sin(theta_ - coneangle_rad / 2)) * scale)));
    cv::Point rightConeEdge(static_cast<int>(std::round((x_ + (sensor_zero_offset_ + searchRadius) * std::cos(theta_ + coneangle_rad / 2)) * scale)),
                            static_cast<int>(std::round((y_ + (sensor_zero_offset_ + searchRadius) * std::sin(theta_ + coneangle_rad / 2)) * scale)));

    cv::line(image, center, leftConeEdge, cv::Scalar(180, 240, 0), 0.09*scale);
    cv::line(image, center, rightConeEdge, cv::Scalar(180, 240, 0), 0.09 * scale);
}


} // namespace ecn
