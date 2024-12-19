#include "localization.h"



Pose::Pose(double x_val, double y_val, double theta_val)
    : x{x_val},
    y{y_val}, 
    theta{theta_val} {}

    // Update the pose based on the odometry equations
vector<double> Pose::updatePose(double microstep_left, double microstep_right, double r_l, double r_r, double e) {
    // Odometry equations
    double delta_D = (r_r * microstep_right + r_l * microstep_left) / 2.0;
    double delta_theta = (r_r * microstep_right - r_l * microstep_left) / e;

    // Update the pose values
    theta += delta_theta;
    x += delta_D * cos(theta + delta_theta / 2.0);
    y += delta_D * sin(theta + delta_theta / 2.0);
    return {x,y,theta};
}

    // Compute wheel angles based on target position
        // kp = proportional gain of the controller
std::pair<double, double> computeWheelAngles(double target_x, double target_y, double r, double L, double kp) {
    // Calculate the Euclidean distance to the target
    double distance = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));
    
    // Calculate the angle error to the target
    double target_angle = atan2(target_y - y, target_x - x);
    double angle_error = target_angle - theta;

    // Normalize angle_error to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    // Compute the wheel control matrix elements
    double thetaL = (kp / r) * distance + (kp * L / r) * angle_error;
    double thetaR = (kp / r) * distance - (kp * L / r) * angle_error;

    // Return the pair of wheel angles
    return std::make_pair(thetaL, thetaR);
}


// MODIFY PATH STRUCTURE TO WORK WITH CLASS ECN::POINT
void followPath(const std::pair<double, double> path[], int path_size, double r, double L, double kp, double r_l, double r_r, double e) {
    int i = 0; // Start at the first point
    while (i < path_size) {
        // Get the target point
        double target_x = path[i].first;     // MODIFY TO Point.x
        double target_y = path[i].second;    // MODIFY TO Point.y

        // Compute wheel angles (implement computeWheelAngles)
        auto [thetaL, thetaR] = computeWheelAngles(target_x, target_y, r, L, kp);

        // HERE WE WILL HAVE STEPPER.MOVETO() AND THE WHILE LOOP OF STEPPER:RUN()

        // Update pose (implement updatePose)
        updatePose(thetaL, thetaR, r_l, r_r, e);

        // Move to the next point
        i = (i + 5 < path_size) ? i + 5 : path_size - 1;
    }
}


