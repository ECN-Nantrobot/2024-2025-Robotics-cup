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


