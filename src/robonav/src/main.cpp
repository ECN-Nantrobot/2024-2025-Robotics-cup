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


enum RobotState { WAITING, INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };

RobotState state = WAITING;

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

#include <tf2_ros/transform_broadcaster.h>

void publishPosition(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher,  
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,               
    const rclcpp::Node::SharedPtr node)                                          

{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = node->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";  // Set the child frame ID

    // Set robot position
    odom_msg.pose.pose.position.x = robot_x / 100.0; // cm to meters
    odom_msg.pose.pose.position.y = 2 - (robot_y / 100.0); // Frame transformation
    odom_msg.pose.pose.position.z = 0.0;

    // Set orientation as quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Publish odometry message
    odom_publisher->publish(odom_msg);

    // ALSO BROADCAST TF TRANSFORM (`odom -> base_link`)
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = odom_msg.header.stamp;
    odom_tf.header.frame_id = "odom"; 
    odom_tf.child_frame_id = "base_link"; 

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

    // Publish the TF transform
    tf_broadcaster->sendTransform(odom_tf);
}



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
    std::cout << "Sent Path:    ";
    for (size_t i = 0; i < path.size() && i < 4; ++i) {
        std::cout << i << ": (" << path[i].x << ", " << path[i].y << ")  ";
    }
    std::cout << " ..." << std::endl;

    // Print the entire path to std
    std::cout << "Full Path: ";
    for (const auto& point : path) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
    std::cout << std::endl;
}

// void sendPath(serial::Serial& ser, const std::vector<ecn::Point>& path)
// {
//     std::string message = "PATH:";
//     for (const auto& point : path) {
//         message += std::to_string(point.x) + "," + std::to_string(point.y) + ";";
//         if (message.length() > 512) { // Send every 512 bytes
//             message += "\n";
//             ser.write(message);
//             std::cout << "Sent Path: " << message << std::endl;
//             message.clear();
//             std::this_thread::sleep_for(std::chrono::milliseconds(5));
//         }
//     }
//     message += "\n";
//     ser.write(message);
//     std::cout << "Sent Path: " << message << std::endl;
// }


void sendSpeedAndPID(serial::Serial& ser, double speed, double P, double I, double D)
{
    std::string message = "SPEED_PID:" + std::to_string(speed) + "," + std::to_string(P) + "," + std::to_string(I) + "," + std::to_string(D) + "\n";
    ser.write(message);
    std::cout << "Sent Speed and PID: " << message << std::endl;
}


ecn::Point pop;

void processCommand(const std::string& command)
{
    std::cout << "Recieved from ESP: " << command;

    if (command.rfind("X:", 0) == 0) {
        std::string position_string = command;
        receivePosition(position_string);

    } else if (command.rfind("STATE:", 0) == 0) {
        std::string stateStr = command.substr(6);
        stateStr.erase(std::remove(stateStr.begin(), stateStr.end(), '\n'), stateStr.end());
        if (stateStr == "INIT")
            state = INIT;
        else if (stateStr == "PATH_PLANNING")
            state = PATH_PLANNING;
        else if (stateStr == "TURN_TO_GOAL")
            state = WAITING;
        else if (stateStr == "TURN_TO_PATH")
            state = WAITING;
        else if (stateStr == "GOAL_REACHED")
            state = GOAL_REACHED;
        std::cout << "ACK:STATE_RECEIVED: " << stateStr << std::endl;

    } else if (command.rfind("CurrenState: ", 0) == 0) {
        std::cout << "Current State: " << command.substr(13) << std::endl;
    }
    else if (command.rfind("POP: ", 0) == 0) {
        std::string popStr = command.substr(5);
        if (sscanf(popStr.c_str(), "%f,%f", &pop.x, &pop.y) == 2) {
            std::cout << "Updated POP -> X: " << pop.x << ", Y: " << pop.y << std::endl;
        } else {
            std::cout << "⚠️ Failed to parse POP data." << std::endl;
        }
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node               = rclcpp::Node::make_shared("velocity_publisher");
    auto velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto path_publisher     = node->create_publisher<nav_msgs::msg::Path>("elastic_band_path", 10);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    // auto odom_subscriber    = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odomCallback);

    serial::Serial ser;
    bool port_opened = false;

    for (int i = 0; i < 10; ++i) {
        std::string port = "/dev/ttyUSB" + std::to_string(i);
        ser.setPort(port);
        ser.setBaudrate(921600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        try {
            ser.open();
            if (ser.isOpen()) {
                std::cout << "Serial port " << port << " opened successfully" << std::endl;
                port_opened = true;
                break;
            }
        } catch (serial::IOException &e) {
            std::cout << "Unable to open port " << port << std::endl;
        }
    }

    if (!port_opened) {
        std::cerr << "Failed to open any serial port." << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give ESP time to start

    ser.write("RESET\n");
    std::cout << "Sent RESET command" << std::endl;

    //-------------------------------------------------------------------------------------

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);
    Point::maze.computeDistanceTransform(); // Precompute distance transform

    Point start;
    Point goal;
    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
    auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    std::vector<Point> goals = { Point(270, 150), Point(40, 40)}; // goal points
    int current_goal_index   = 1;   // Keep track of which goal the robot is targeting
    start                    = goals[0];
    goal                    = goals[current_goal_index];
    std::vector<double> target_thetas(goals.size(), 0); // Initialize target thetas
    target_thetas = { 45 *M_PI/180, 0 };


    Robot robot(Point::maze, start.x, start.y, target_thetas[0] * M_PI / 180, 18.5, 3, 5, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D
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

    float distance_to_goal                = robot.distanceToGoal(goal);
    int counter_set_eb_path = 0;
    int set_eb_path_counter_limit_default = 1;
    int set_eb_path_counter_limit         = set_eb_path_counter_limit_default;
    int eb_comp_inarow_default            = 1;
    int eb_comp_inarow                    = eb_comp_inarow_default;
    double path_difference_front = 0.0;
    double path_difference_end   = 0.0;
    double path_difference       = 0.0;
    double path_difference_check_limit = 0.0;
    
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

    elastic_band.updatePath(astar_path);
    elastic_band.runFullOptimization(start, goal);


    std::cout << "Waiting for ESP reset ... " << std::endl;
    while (true) {
        if (ser.available()) {
            std::string result = ser.readline();
            if (result.find("ESP Initialized!") != std::string::npos) {
                std::cout << "ESP Initialization confirmed!" << std::endl;
                break;
            }
        }
    }
    std::cout << std::endl; // Move to the next line after the loop ends

    publishPath(elastic_band.getSmoothedPath(), path_publisher, node);
    sendSpeedAndPID(ser, 3.0, 5, 0.01, 0.5);
    sendGoals(ser, goals, target_thetas);
    sendPath(ser, elastic_band.getSmoothedPath());


    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ser.write("STATE:INIT\n");
    std::cout << "Sent INIT state" << std::endl;

    std::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;

    int navigation_counter = 0;

    rclcpp::Rate loop_rate(20); // 20 Hz, 50ms per loop
    

    ////////////////////////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////////////////////
    while (rclcpp::ok()) {
        
        int max_reads = 10;

        // Prevent infinite looping
        // std::string latest_command;
        // while (ser.available() && max_reads > 0) {
        //     latest_command = ser.readline(); // Keep only the latest message
        //     max_reads--;
        //     }

        //     if (!latest_command.empty()) {
        //         processCommand(latest_command);

        while (ser.available() && max_reads > 0) {
            std::string command = ser.readline();
            processCommand(command);
            max_reads--;
        }

        robot.setPose(robot_x, robot_y, robot_theta);

        switch (state) {
        case WAITING:
            std::cout << "State: WAITING" << std::endl;

            break;

        case PATH_PLANNING:
            std::cout << "State: PATH_PLANNING" << std::endl;

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
            std::cout << "State: NAVIGATION to: (" << goal.x << ", " << goal.y << ")" << std::endl;

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

                    // publishPath(elastic_band.getSmoothedPath(), path_publisher, node);
                    sendPath(ser, elastic_band.getSmoothedPath());

                } else {
                }
            }

            navigation_counter++;

            if (navigation_counter >= 20) {
                state = PATH_PLANNING;
                navigation_counter = 0;
            }

            break;

        case GOAL_REACHED:
            std::cout << "State: GOAL_REACHED" << std::endl;

            if (current_goal_index < goals.size() - 1) {
                current_goal_index++;
                goal = goals[current_goal_index];
                state = PATH_PLANNING;
                std::cout << "New goal: (" << goal.x << ", " << goal.y << ")" << std::endl;
            } else {
                std::cout << "All goals reached!" << std::endl;
                state = WAITING;
            }

            break;
        }

        publishPosition(odom_publisher, tf_broadcaster, node);

        loop_rate.sleep();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }

    rclcpp::shutdown();

    return 0;
}
