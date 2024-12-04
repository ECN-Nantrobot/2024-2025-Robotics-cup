#include <iostream>
#include "localization.h"

int main() {
    // Initial pose (x, y, theta)
    Pose robot_pose(0.0, 0.0, 0.0);
    
    // Example values for microstep_left and microstep_right
    double microstep_left = 1.0;  // Increment for left wheel
    double microstep_right = 1.2; // Increment for right wheel
    double r_l = 0.05;            // Radius of left wheel
    double r_r = 0.05;            // Radius of right wheel
    double e = 0.2;               // Distance between wheels
    while(1){
    
        // Update the pose of the robot
        robot_pose.updatePose(microstep_left, microstep_right, r_l, r_r, e);

        // Output the updated pose
        std::cout << "Updated X: " << robot_pose.getX() << std::endl;
        std::cout << "Updated Y: " << robot_pose.getY() << std::endl;
        std::cout << "Updated Theta: " << robot_pose.getTheta() << std::endl;


        double kp = 1.0; // Proportional gain
        // Compute the wheel angles
        std::pair<double, double> wheel_angles = robot_pose.computeWheelAngles(target_x, target_y, r, L, kp);
    
        // Output the results
        std::cout << "Theta Left (thetaL): " << wheel_angles.first << std::endl;
        std::cout << "Theta Right (thetaR): " << wheel_angles.second << std::endl;
    }
    return 0;
}
