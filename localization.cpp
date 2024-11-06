#include <iostream>
#include <cmath>

class Pose {
private:
    double x;      // x coordinate
    double y;      // y coordinate
    double theta;  // orientation angle

public:
    // Constructor to initialize pose with x, y, and theta values
    Pose(double x_val, double y_val, double theta_val)
        : x(x_val), y(y_val), theta(theta_val) {}

    // Getter methods
    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }

    // Update the pose based on the odometry equations
    void updatePose(double microstep_left, double microstep_right, double r_l, double r_r, double e) {
        // Odometry equations
        double delta_D = (r_r * microstep_right + r_l * microstep_left) / 2.0;
        double delta_theta = (r_r * microstep_right - r_l * microstep_left) / e;

        // Update the pose values
        theta += delta_theta;
        x += delta_D * cos(theta + delta_theta / 2.0);
        y += delta_D * sin(theta + delta_theta / 2.0);
    }
};

int main() {
    // Initial pose (x, y, theta)
    Pose robot_pose(0.0, 0.0, 0.0);

    // Example values for microstep_left and microstep_right
    double microstep_left = 1.0;  // Increment for left wheel
    double microstep_right = 1.2; // Increment for right wheel
    double r_l = 0.05;            // Radius of left wheel
    double r_r = 0.05;            // Radius of right wheel
    double e = 0.2;               // Distance between wheels

    // Update the pose of the robot
    robot_pose.updatePose(microstep_left, microstep_right, r_l, r_r, e);

    // Output the updated pose
    std::cout << "Updated X: " << robot_pose.getX() << std::endl;
    std::cout << "Updated Y: " << robot_pose.getY() << std::endl;
    std::cout << "Updated Theta: " << robot_pose.getTheta() << std::endl;

    return 0;
}