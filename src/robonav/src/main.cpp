#include <a_star.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <elastic_bands.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <maze.h>
#include <point.h>
#include <position.h>

#include "point.h"
#include "robot.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <serial/serial.h>


using namespace std;
using namespace ecn;


enum RobotState { INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };

void publishPath(const std::vector<Point>& elastic_path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher, const rclcpp::Node::SharedPtr& node)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = node->now();
    path_msg.header.frame_id = "odom"; 

    for (const auto& point : elastic_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp       = path_msg.header.stamp;
        pose.header.frame_id    = "odom";
        pose.pose.position.x    = point.x/100.0;  // Convert from cm to m
        pose.pose.position.y    = 2 - point.y/100.0; // Conversion cm to m and frame change
        pose.pose.position.z    = 0.0;
        pose.pose.orientation.w = 1.0; // Default orientation

        path_msg.poses.push_back(pose);
    }

    path_publisher->publish(path_msg);
}

double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;


// void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     // Get `x` and `y`, convert from meters to cm
//     robot_x = msg->pose.pose.position.x * 100;
//     robot_y = 200 - msg->pose.pose.position.y * 100;

//     // Correctly Convert Quaternion to Yaw (Theta)
//     tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw); // Extract yaw angle

//     robot_theta = - yaw; // Now theta (yaw) is correct

//     RCLCPP_INFO(rclcpp::get_logger("Odometry"), "Odom: x=%.2f, y=%.2f, theta=%.2f°", robot_x, robot_y, robot_theta * 180 / M_PI);
// }

// ROS dt 
rclcpp::Time last_time;
double getDt(const rclcpp::Node::SharedPtr& node)
{
    static rclcpp::Time last_time = node->now();
    rclcpp::Time now = node->now();
    double dt = (now - last_time).seconds();
    last_time = now;
    return dt;
}

// C++ dt
// double getDt()
// {
//     static auto last_time                 = std::chrono::steady_clock::now();
//     auto now                              = std::chrono::steady_clock::now();
//     std::chrono::duration<double> elapsed = now - last_time;
//     last_time                             = now;
//     // std::cout << "dt: " << elapsed.count() << " seconds" << std::endl;
//     return elapsed.count();
// }


void receivePosition(std::string result) {
    
    if (sscanf(result.c_str(), "X: %lf, Y: %lf, Theta: %lf", &robot_x, &robot_y, &robot_theta) == 3) {
        // std::cout << "Updated Position -> X: " << robot_x << ", Y: " << robot_y << ", Theta: " << robot_theta << std::endl;
    } else {
        std::cout << "⚠️ Failed to parse position data." << std::endl;
    }
}

void sendGoals(serial::Serial& ser, const vector<Point>& goals, const vector<double>& target_thetas)
{
    string message = "GOALS:";
    for (size_t i = 0; i < goals.size(); i++) {
        message += to_string(goals[i].x) + "," + to_string(goals[i].y) + "," + to_string(target_thetas[i]);
        if (i < goals.size() - 1)
            message += ";"; // Separate goals
    }
    message += "\n";
    ser.write(message);
    cout << "Sent Goals: " << message << endl;
}

void sendPath(serial::Serial& ser, const std::vector<ecn::Point>& path)
{
    std::string message = "PATH:";
    for (const auto& point : path) {
        message += std::to_string(point.x) + "," + std::to_string(point.y) + ";";
    }
    message += "\n";
    ser.write(message);
    std::cout << "Sent Path:" << std::endl;
    for (size_t i = 0; i < path.size() && i < 5; ++i) {
        std::cout << "Path " << i << ": (" << path[i].x << ", " << path[i].y << ")" << std::endl;
    }
    std::cout << "..." << std::endl;
}



void processCommand(const std::string& command)
{
    std::cout << "Recieved from ESP: " << command;

    if (command.rfind("X:", 0) == 0) {
        std::string position_string = command;
        receivePosition(position_string);
    }
        // Use the receivePosition function to process and save to robot_x, robot_y, robot_theta

}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node               = rclcpp::Node::make_shared("velocity_publisher");
    auto velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto path_publisher     = node->create_publisher<nav_msgs::msg::Path>("elastic_band_path", 10);
    // auto odom_subscriber    = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odomCallback);

    last_time = node->now();

    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    try {
        ser.open();
        if(ser.isOpen())
        {
          RCLCPP_INFO(node->get_logger(), "Serial port opened successfully");
        }
    } catch (serial::IOException &e) {
       RCLCPP_ERROR(node->get_logger(), "Unable to open port: %s", e.what());
    }

    ser.write("RESET\n");
    std::cout << "Sent RESET command" << std::endl;


    //-------------------------------------------------------------------------------------
    RobotState state = PATH_PLANNING;

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);
    Point::maze.computeDistanceTransform(); // Precompute distance transform

    Point start;
    Point goal;
    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));

    std::vector<Point> goals = { Point(270, 150), Point(40, 40)}; // goal points
    int current_goal_index   = 1;   // Keep track of which goal the robot is targeting
    start                    = goals[0];
    goal                    = goals[current_goal_index];
    std::vector<double> target_thetas(goals.size(), 0); // Initialize target thetas
    target_thetas = { 45 *M_PI/180, 0 };



    int counter_set_eb_path = 0;

    Robot robot(Point::maze, start.x, start.y, target_thetas[0] * M_PI / 180, 33, 5, 10, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D
    robot.setIsStarting(true);  // Enable gradual start
    robot.setPose(goals[0].x, goals[0].y, target_thetas[0] * M_PI / 180);
    robot.setTargetTheta(target_thetas[0]);

    robot_x     = robot.getX();
    robot_y     = robot.getY();
    robot_theta = robot.getTheta();

    std::vector<Position> astar_path;
    std::vector<Position> astar_path_previous;
    ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.resetOptimization(); // Setzt die Optimierung zurück

    int t          = 0;

    float distance_to_goal                = robot.distanceToGoal(goal);
    int set_eb_path_counter_limit_default = 1;
    int set_eb_path_counter_limit         = set_eb_path_counter_limit_default;
    int eb_comp_inarow_default            = 1;
    int eb_comp_inarow                    = eb_comp_inarow_default;
    double path_difference_front = 0.0;
    double path_difference_end   = 0.0;
    double path_difference       = 0.0;
    double path_difference_check_limit = 0.0;
    


    rclcpp::sleep_for(std::chrono::seconds(3));

    goal = goals[current_goal_index];

    Point::maze.computeDistanceTransform();
    cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
    start_p    = Position(static_cast<int>(robot.getX() * Point::maze.resize_for_astar), static_cast<int>(robot.getY() * Point::maze.resize_for_astar));
    goal_p     = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
    astar_path = Astar(start_p, goal_p);

    for (auto& position : astar_path) {
        position           = position * (1 / Point::maze.resize_for_astar);
        astar_path.front() = Position(start);
        astar_path.back()  = Position(goal);
    }

    elastic_band.updatePath(astar_path);
    elastic_band.runFullOptimization(start, goal);



    // Wait until serial sends "ESP Initialized!"
    while (rclcpp::ok()) {
        if (ser.available()) {
            std::string result = ser.read(ser.available());
            RCLCPP_INFO(node->get_logger(), "From ESP: %s", result.c_str());
            std::cout << result << std::endl;
            if (result.find("ESP Initialized!") != std::string::npos) {
                RCLCPP_INFO(node->get_logger(), "ESP Initialization confirmed!");
                break;
            }
        }
        rclcpp::spin_some(node); // Allow ROS 2 callbacks to be processed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    std::cout << "OKEEE Letsgo" << std::endl;

    publishPath(elastic_band.getSmoothedPath(), path_publisher, node);
    sendGoals(ser, goals, target_thetas);
    sendPath(ser, elastic_band.getSmoothedPath());

    while (ser.available()) {
        std::string command = ser.readline(); // Wartet auf eine vollständige Zeile (\n)
        processCommand(command);
    }

    ser.write("STATE:INIT\n");
    std::cout << "Sent INIT state" << std::endl;


    std::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;


    // double dt = 0.05;
    rclcpp::Rate loop_rate(3000); // 20 Hz, 50ms per loop

    ////////////////////////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////////////////////
    while (rclcpp::ok()) {

    // double dt = getDt(node);

        while (ser.available()) {
            std::string command = ser.readline(); // Wartet auf eine vollständige Zeile (\n)
            processCommand(command);
            std::cout << "REEEEEEEEEEEEEEEEEEEEEEEEEEEEAAAAAAAAAAAAAAAADing" << std::endl;
        }

        // receivePosition(ser, robot_x, robot_y, robot_theta);
        robot.setPose(robot_x, robot_y, robot_theta);

        switch (state) {
        case PATH_PLANNING:
            std::cout << "State: PLANNING" << std::endl;

            // Recalculate A* path
            // Point::maze.computeDistanceTransform();
            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p    = Position(static_cast<int>(robot.getX() * Point::maze.resize_for_astar), static_cast<int>(robot.getY() * Point::maze.resize_for_astar));
            goal_p     = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
            astar_path = Astar(start_p, goal_p);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = Position(start);
                astar_path.back()  = Position(goal);
            }

            // Check the difference in the A* path
            path_difference_front = 0.0;
            path_difference_end   = 0.0;
            path_difference       = 0.0;
            // path_difference_check_limit = astar_path.size() < 25 ? astar_path.size() : astar_path.size() / 2;
            path_difference_check_limit = astar_path.size() / 2;
            if (astar_path_previous.size() != 0) {
                for (size_t i = 1 + path_difference_check_limit; i < astar_path.size(); i += 2) {
                    double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                    double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                    path_difference_front += sqrt(dx * dx + dy * dy);
                }
                path_difference_front /= astar_path.size();
                // std ::cout << "A* path diff front: %: " << path_difference_front;

                for (size_t i = 1; i < astar_path.size() - path_difference_check_limit; i += 2) {
                    double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                    double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                    path_difference_end += sqrt(dx * dx + dy * dy);
                }
                path_difference_end /= astar_path.size();
                // std ::cout << " A* path diff end: %: " << path_difference_end;
            } else {
                // std::cout << "astar_path_previous.size() = " << astar_path_previous.size() << std::endl;
            }
            path_difference = path_difference_front + path_difference_end;
            if (path_difference_front > 3) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                // std::cout << "    -> A* Path Front, set eb" << std::endl;
                elastic_band.resetOptimization();
                counter_set_eb_path       = 0;
                set_eb_path_counter_limit = 1;
                eb_comp_inarow            = 1;
            } else if (path_difference_end > 4) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                // std::cout << "    -> A* Path Back, set eb" << std::endl;
                set_eb_path_counter_limit = 2;
                counter_set_eb_path       = 0;
                elastic_band.resetOptimization();
            } else if (path_difference >= 3.1) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                // std::cout << "    -> A* Path FULL, set eb" << std::endl;
                elastic_band.resetOptimization();
                counter_set_eb_path       = 0;
                set_eb_path_counter_limit = 1;
                eb_comp_inarow            = 1;
                elastic_band.setMaxInterations(6);
            } else if (path_difference > 2.6 && path_difference < 3 || astar_path_previous.size() == 0) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                // std::cout << "    -> A* Path MEDIUM, set eb" << std::endl;
                set_eb_path_counter_limit = 3;
                counter_set_eb_path       = 0;
                elastic_band.resetOptimization();
            } else {
                std ::cout << "" << std::endl;
            }

            state = NAVIGATION;
            [[fallthrough]];


        case NAVIGATION:
            std::cout << "State: NAVIGATION, goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

            elastic_band.optimize(start, goal);

            if (elastic_band.errorCheck() == true) {
                // std::cout << "EB Optimization error -> PATH PLANNING." << std::endl;
                elastic_band.resetOptimization();
                state = PATH_PLANNING;
                break;
            }

            if (elastic_band.isOptimizationComplete()) {
                // std::cout << "Elastic Band optimization completed";
                counter_set_eb_path++;
                elastic_band.resetOptimization();

                if (counter_set_eb_path > set_eb_path_counter_limit) {
                    elastic_band.generateSmoothedPath(0.8f, 21, 1.2f); // 0.08 ms

                    set_eb_path_counter_limit = set_eb_path_counter_limit_default;
                    eb_comp_inarow            = eb_comp_inarow_default;

                    publishPath(elastic_band.getSmoothedPath(), path_publisher, node);
                    sendPath(ser, elastic_band.getSmoothedPath());

                } else {
                }
            }

            // robot.followPath(elastic_band.getSmoothedPath(), Point::maze, dt);

            break;

        }


        // auto msg = geometry_msgs::msg::Twist();
        // msg.linear.x  = robot.getLinearVelocity() /100;  // Replace with your velocity computation
        // msg.angular.z = - robot.getAngularVelocity(); // Replace with your rotation computation
        // // RCLCPP_INFO(node->get_logger(), "Publishing velocity: linear.x = %f, angular.z = %f", msg.linear.x, msg.angular.z);
        // velocity_publisher->publish(msg);

        // rclcpp::spin_some(node); // Allow ROS 2 callbacks to be processed
        loop_rate.sleep();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // robot.updatePosition(dt);
        // robot.setPose(robot_x, robot_y, robot_theta);

   }

    rclcpp::shutdown();

    return 0;
}
