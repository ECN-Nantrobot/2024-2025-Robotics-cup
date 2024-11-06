#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;
class Pose {
private:
    double x{};      // x coordinate
    double y{};      // y coordinate
    double theta{};  // orientation angle

public:
    // Constructor to initialize pose with x, y, and theta values
    Pose(double x_val, double y_val, double theta_val); 

    // Getter methods
    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }

    // Update the pose based on the odometry equations
    vector<double> updatePose(double microstep_left, double microstep_right, double r_l, double r_r, double e);
};

#endif  // LOCALIZATION_H