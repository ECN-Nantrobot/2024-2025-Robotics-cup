#include <a_star.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <elastic_bands.h>
#include <filesystem>
#include <fstream>
#include <iomanip> // For std::fixed and std::setprecision
#include <iostream>
#include <maze.h>
#include <point.h>
#include <position.h>
#include <sstream>         // For std::ostringstream
#include <yaml-cpp/yaml.h> // Include the YAML library

#include "point.h"
#include "robot.h"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <serial/serial.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>


#include "std_msgs/msg/float64.hpp"
#include <csignal>
#include <vector>

using namespace std;
using namespace ecn;

enum RobotState { WAITING, INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };

RobotState state = WAITING;

Robot robot(Point::maze, 0, 0, 0, 18.5, 8, 25, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D

double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
ecn::Point pop;


void publishElasticbandPath(const std::vector<Point>& elastic_path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher, const rclcpp::Node::SharedPtr& node)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = node->now();
    path_msg.header.frame_id = "odom";

    for (const auto& point : elastic_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp       = path_msg.header.stamp;
        pose.header.frame_id    = "odom";
        pose.pose.position.x    = point.x / 100.0;     // Convert from cm to m
        pose.pose.position.y    = 2 - point.y / 100.0; // Conversion cm to m and frame change
        pose.pose.position.z    = 0.0;
        pose.pose.orientation.w = 1.0; // Default orientation

        path_msg.poses.push_back(pose);
    }

    path_publisher->publish(path_msg);
}

// Function to publish A* path as nav_msgs/Path
void publishAStarPath(const std::vector<Position>& astar_path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher, const rclcpp::Node::SharedPtr& node)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = node->now();
    path_msg.header.frame_id = "odom"; // Set the appropriate frame

    path_msg.poses.clear(); // Clear any residual data

    for (const auto& position : astar_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp    = path_msg.header.stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = position.x / 100.0;     // Convert cm to meters
        pose.pose.position.y = 2 - position.y / 100.0; // Verify this transformation
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, -robot_theta); // Verify robot_theta value
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose);
    }

    path_publisher->publish(path_msg);
}

// Function to publish Elastic Band circles as visualization_msgs/Marker
void publishElasticBandCircles(const std::vector<Point>& eb_path, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& marker_publisher, const rclcpp::Node::SharedPtr& node)
{
    visualization_msgs::msg::Marker marker;
    marker.header.stamp    = node->now();
    marker.header.frame_id = "odom"; // Set the appropriate frame
    marker.ns              = "elastic_band_circles";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action          = visualization_msgs::msg::Marker::ADD;

    for (const auto& point : eb_path) {
        geometry_msgs::msg::Point sphere_center;
        sphere_center.x = point.x / 100.0; // Convert cm to meters
        sphere_center.y = 2 - point.y / 100.0;
        sphere_center.z = 0.0;
        marker.points.push_back(sphere_center);

        // Set the size of each circle based on point.radius
        marker.scale.x = point.radius / 100.0; // Convert cm to meters
        marker.scale.y = point.radius / 100.0;
        marker.scale.z = point.radius / 100.0;

        // Use the color from point.colour (cv::Scalar(255, 50, 50))
        std_msgs::msg::ColorRGBA sphere_color;
        sphere_color.r = point.colour[2] / 255.0; // OpenCV uses BGR, normalize to [0, 1]
        sphere_color.g = point.colour[1] / 255.0;
        sphere_color.b = point.colour[0] / 255.0;
        sphere_color.a = 0.03; // opacity
        marker.colors.push_back(sphere_color);
    }

    marker_publisher->publish(marker);
}


void publishPosition(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, const rclcpp::Node::SharedPtr node)

{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = node->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link"; // Set the child frame ID

    // Set robot position
    odom_msg.pose.pose.position.x = robot_x / 100.0;       // cm to meters
    odom_msg.pose.pose.position.y = 2 - (robot_y / 100.0); // Frame transformation
    odom_msg.pose.pose.position.z = 0.0;

    // Set orientation as quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, -robot_theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Publish odometry message
    odom_publisher->publish(odom_msg);

    // ALSO BROADCAST TF TRANSFORM (`odom -> base_link`)
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp    = odom_msg.header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id  = "base_link";

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;

    // Publish the TF transform
    tf_broadcaster->sendTransform(odom_tf);
}

void publishPoint(const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr& publisher, const rclcpp::Node::SharedPtr& node, const ecn::Point& point)
{
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp    = node->now();
    point_msg.header.frame_id = "odom"; // Set your frame_id

    // Set the coordinates of the point
    point_msg.point.x = point.x / 100;       // X coordinate in meters
    point_msg.point.y = 2 - (point.y / 100); // Y coordinate in meters
    point_msg.point.z = 0.0;

    publisher->publish(point_msg);
    // RCLCPP_INFO(node->get_logger(), "Publishing point at: (%.2f, %.2f, %.2f)", point_msg.point.x, point_msg.point.y, point_msg.point.z);
}

void publishOccupancyGrid(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& publisher, cv::Mat& origina_image)
{
    cv::Mat image = origina_image.clone();

    cv::flip(image, image, 0); // Flip the image on the horizontal axis

    if (image.channels() != 1) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY); // Convert to grayscale (if not already)
    }

    if (image.type() == CV_16U) {
        cv::Mat im_8bit;
        image.convertTo(im_8bit, CV_8UC1, 1.0 / 256); // Normalize 16-bit to 8-bit range
        image = im_8bit;
    }

    // Convert OpenCV image to OccupancyGrid data
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution           = 0.01; // meters per pixel
    grid.info.width                = image.cols;
    grid.info.height               = image.rows;
    grid.info.origin.position.x    = 0.0;
    grid.info.origin.position.y    = 0.0;
    grid.info.origin.position.z    = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.header.frame_id           = "odom";

    std::vector<int8_t> data;
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            uint8_t pixel_value = image.at<uchar>(i, j);
            if (pixel_value < 120) // Occupied
            {
                data.push_back(100);       // 100 represents occupied
            } else if (pixel_value >= 120) // Free
            {
                data.push_back(0); // 0 represents free space
            } else                 // Unknown area
            {
                data.push_back(-1); // -1 represents unknown space
            }
        }
    }

    grid.data = data;
    publisher->publish(grid);
}


void receivePosition(std::string result)
{

    if (sscanf(result.c_str(), "X: %lf, Y: %lf, Theta: %lf", &robot_x, &robot_y, &robot_theta) == 3) {
        // std::cout << "Updated Position -> X: " << robot_x << ", Y: " << robot_y << ", Theta: " << robot_theta << std::endl;
    } else {
        std::cout << "⚠️ Failed to parse position data." << std::endl;
    }
}

void sendGoals(serial::Serial& ser)
{
    std::cout << "Sending goals to ESP:" << std::endl;
    string message = "GOALS:";
    for (size_t i = 0; i < robot.goals.size(); i++) {
        std::ostringstream goal_oss;                    // stringstream
        goal_oss << std::fixed << std::setprecision(2); // Set fixed-point format with 2 decimal places

        goal_oss << robot.goals[i].point.x << "," << robot.goals[i].point.y << "," << robot.goals[i].theta << ";";
        message += goal_oss.str();
        std::cout << goal_oss.str() << std::endl;
    }
    message += "\n";
    ser.write(message);
}

void sendPath(serial::Serial& ser, const std::vector<ecn::Point>& path)
{
    auto path_send_start_time = std::chrono::steady_clock::now(); // Start timing

    int path_sending_limit = 3;                                              // Send x points at a time
    size_t path_size       = std::min(path.size(), static_cast<size_t>(32)); // total points to save

    int iterations = 0;
    // Loop through the entire path in chunks (in this case, 2 points at a time)
    for (size_t i = 0; i < path_size; i += path_sending_limit) {
        std::string message;
        if (i + path_sending_limit >= path_size) {
            message = "PATHEND:"; // Use "PATHEND:" for the last batch
        } else {
            message = (i == 0) ? "PATH:" : "PT:"; // Use "PATH:" for the first batch, "PT:" for subsequent ones
        }

        size_t max_points = std::min(path_size - i, static_cast<size_t>(path_sending_limit)); // Limit the number of points in each chunk

        std::ostringstream path_oss;
        path_oss << std::fixed << std::setprecision(2); // Set fixed-point format with 2 decimal places

        // Add points to the message
        for (size_t j = i; j < i + max_points; ++j) {
            // Format x and y to two decimal places
            path_oss << path[j].x << "," << path[j].y << ";";
        }

        message += path_oss.str(); // Add the formatted points to the message
        message += "\n";           // Add a newline to mark the end of the message

        ser.write(message); // Send the message over serial
        iterations++;
        // std::cout << message;                                      // Print the message to std
        std::this_thread::sleep_for(std::chrono::milliseconds(6)); // Optional: wait before sending the next chunk
    }
    // std::cout << std::endl;
    std::cout << iterations << " of " << path_size / path_sending_limit << " PATH chunks sent" << std::endl;

    auto path_send_end_time   = std::chrono::steady_clock::now(); // End timing
    double path_send_duration = std::chrono::duration_cast<std::chrono::milliseconds>(path_send_end_time - path_send_start_time).count();
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;

    std::cout << "Path sending time: " << path_send_duration << " ms" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;

    std::cout << "" << std::endl;
}

void sendSpeedAndPID(serial::Serial& ser, double speed, double P, double I, double D)
{
    std::string message = "SPEED_PID:" + std::to_string(speed) + "," + std::to_string(P) + "," + std::to_string(I) + "," + std::to_string(D) + "\n";
    ser.write(message);
    std::cout << "Sent Speed and PID: " << message << std::endl;
}


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
        else if (stateStr == "WAIT")
            state = WAITING;
        else if (stateStr == "TURN_TO_GOAL")
            state = WAITING;
        else if (stateStr == "TURN_TO_PATH")
            state = WAITING;
        else if (stateStr == "GOAL_REACHED")
            state = GOAL_REACHED;
        std::cout << "ACK:STATE_RECEIVED: [" << stateStr << "]" << std::endl;

    } else if (command.rfind("CurrenState: ", 0) == 0) {
        std::cout << "Current State: " << command.substr(13) << std::endl;
    } else if (command.rfind("POP:", 0) == 0) {
        std::string popStr = command.substr(4);
        if (sscanf(popStr.c_str(), "%f,%f", &pop.x, &pop.y) == 2) {}
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loadGoalsFromFile(const std::string& filename, std::vector<ecn::Pose>& goals, double& wheel_distance, double& speed, double& kp, double& ki, double& kd)
{
    YAML::Node yaml_config = YAML::LoadFile(filename);

    // Load robot parameters
    wheel_distance = yaml_config["robot"]["wheel_distance"].as<double>();
    speed          = yaml_config["robot"]["speed"].as<double>();
    kp             = yaml_config["robot"]["pid"]["kp"].as<double>();
    ki             = yaml_config["robot"]["pid"]["ki"].as<double>();
    kd             = yaml_config["robot"]["pid"]["kd"].as<double>();

    // Load goals
    goals.clear();
    for (const auto& goal : yaml_config["goals"]) {
        goals.emplace_back(goal["x"].as<float>(), goal["y"].as<float>(), goal["theta"].as<float>());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node               = rclcpp::Node::make_shared("velocity_publisher");
    auto velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto path_publisher     = node->create_publisher<nav_msgs::msg::Path>("elastic_band_path", 10);
    auto odom_publisher     = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    // auto odom_subscriber    = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odomCallback);
    auto point_publisher    = node->create_publisher<geometry_msgs::msg::PointStamped>("point_topic", 10);
    auto occupancy_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    auto tf_broadcaster     = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto elastic_band_path_pub    = node->create_publisher<nav_msgs::msg::Path>("/elastic_band_path", 10);
    auto astar_path_pub           = node->create_publisher<nav_msgs::msg::Path>("/astar_path", 10);
    auto elastic_band_circles_pub = node->create_publisher<visualization_msgs::msg::Marker>("/elastic_band_circles", 10);
    auto loop_time_publisher      = node->create_publisher<std_msgs::msg::Float64>("loop_execution_time", 10);


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
        } catch (serial::IOException& e) {
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

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p.png"); // CHOOSE WHICH MAZE YOU WANT TO USE

    Point::maze.load(filename_maze);
    Point::maze.computeDistanceTransform(); // Precompute distance transform

    double wheel_distance, speed, kp, ki, kd;

    try {
        loadGoalsFromFile("/home/pi/2024-2025-Robotics-cup/src/robonav/config/goals.yaml", robot.goals, wheel_distance, speed, kp, ki, kd);
    } catch (const std::exception& e) {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
        return 1;
    }

    robot.setSpeed(speed);
    robot.setPID(kp, ki, kd);
    robot.setWheelBase(wheel_distance);
    robot.setPose(robot.goals[0].x(), robot.goals[0].y(), robot.goals[0].theta * M_PI / 180);

    Point start = robot.goals[0];
    std::cout << "Initial Robot Pose: (" << robot.goals[0].point.x << ", " << robot.goals[0].point.y << ", " << robot.goals[0].theta << ")" << std::endl;
    std::cout << "Start Point: (" << start.x << ", " << start.y << ")" << std::endl;
    Position start_p = Position(static_cast<int>(start.x), static_cast<int>(start.y));
    Position goal_p  = Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));
    Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));

    robot_x     = robot.getX();
    robot_y     = robot.getY();
    robot_theta = robot.getTheta();

    std::vector<Position> astar_path;
    std::vector<Position> astar_path_previous;
    ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.resetOptimization(); // Setzt die Optimierung zurück

    float distance_to_goal                = robot.distanceToGoal(robot.goals[robot.goal_index]);
    int counter_set_eb_path               = 0;
    int set_eb_path_counter_limit_default = 1;
    int set_eb_path_counter_limit         = set_eb_path_counter_limit_default;
    int eb_comp_inarow_default            = 1;
    int eb_comp_inarow                    = eb_comp_inarow_default;
    double path_difference_front          = 0.0;
    double path_difference_end            = 0.0;
    double path_difference                = 0.0;
    double path_difference_check_limit    = 0.0;

    // Point::maze.computeDistanceTransform();
    cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
    astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);
    for (auto& position : astar_path) {
        position           = position * (1 / Point::maze.resize_for_astar);
        astar_path.front() = start_p;
        astar_path.back()  = goal_p;
    }


    elastic_band.updatePath(astar_path);
    elastic_band.runFullOptimization(robot.getPosition(), robot.goals[robot.goal_index]);


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


    publishElasticbandPath(elastic_band.getSmoothedPath(), path_publisher, node);
    publishOccupancyGrid(occupancy_grid_pub, Point::maze.im);

    sendSpeedAndPID(ser, speed, kp, ki, kd);
    sendGoals(ser);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    sendPath(ser, elastic_band.getSmoothedPath());

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ser.write("STATE:INIT\n");
    std::cout << "Sent INIT state" << std::endl;

    std::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;

    int navigation_counter = 0;

    rclcpp::Rate loop_rate(20); // 20 Hz, 50ms per loop

    int send_path_counter = 0;
    ////////////////////////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////////////////////
    while (rclcpp::ok()) {

        auto loop_start_time = std::chrono::steady_clock::now(); // Initialize loop start time


        int max_reads = 15;

        while (ser.available() && max_reads > 0) {
            std::string command = ser.readline();
            processCommand(command);
            max_reads--;
        }

        robot.setPose(robot_x, robot_y, robot_theta);

        switch (state) {
        case WAITING: std::cout << "State: WAITING" << std::endl; break;

        case INIT:
            std::cout << "State: INIT" << std::endl;

            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p    = Position(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
            goal_p     = Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));
            astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = start_p;
                astar_path.back()  = goal_p;
            }


            elastic_band.updatePath(astar_path);
            elastic_band.runFullOptimization(robot.getPosition(), robot.goals[robot.goal_index]);

            publishElasticbandPath(elastic_band.getSmoothedPath(), path_publisher, node);
            sendPath(ser, elastic_band.getSmoothedPath());


            state = WAITING;


            break;

        case PATH_PLANNING:
            std::cout << "State: PATH_PLANNING-------" << std::endl;

            // Recalculate A* path
            // Point::maze.computeDistanceTransform();
            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p = Position(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
            goal_p  = Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));

            astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = start_p;
                astar_path.back()  = goal_p;
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


            elastic_band.updatePath(astar_path);


            state = NAVIGATION;
            [[fallthrough]];


        case NAVIGATION:
            std::cout << "State: NAVIGATION to: (" << robot.goals[robot.goal_index].point.x << ", " << robot.goals[robot.goal_index].point.y << ")" << std::endl;


            elastic_band.optimize(robot.getPosition(), robot.goals[robot.goal_index]);

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
                    elastic_band.generateSmoothedPath(3.0f, 21, 1.2f); // 0.08 ms

                    set_eb_path_counter_limit = set_eb_path_counter_limit_default;
                    eb_comp_inarow            = eb_comp_inarow_default;

                    send_path_counter++;
                    if (send_path_counter >= 5) {
                        publishElasticbandPath(elastic_band.getSmoothedPath(), path_publisher, node);
                        sendPath(ser, elastic_band.getSmoothedPath());
                        send_path_counter = 0;
                    }

                } else {
                }
            }

            navigation_counter++;

            if (navigation_counter >= 15) {
                state              = PATH_PLANNING;
                navigation_counter = 0;
            }

            break;

        case GOAL_REACHED:
            std::cout << "State: GOAL_REACHED" << std::endl;

            if (robot.goal_index < robot.goals.size() - 1) {
                robot.goal_index++;
                std::cout << "New goal: (" << robot.goals[robot.goal_index].point.x << ", " << robot.goals[robot.goal_index].point.y << ")" << std::endl;
            } else {
                std::cout << "All goals reached!" << std::endl;
            }

            state = WAITING;


            break;
        }

        static auto last_clone_time = std::chrono::steady_clock::now();
        auto current_time           = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() >= 5) {
            last_clone_time = current_time;
            publishOccupancyGrid(occupancy_grid_pub, Point::maze.im);
        }

        publishPosition(odom_publisher, tf_broadcaster, node);
        publishPoint(point_publisher, node, pop);
        publishAStarPath(astar_path, astar_path_pub, node);
        publishElasticBandCircles(elastic_band.getSmoothedPath(), elastic_band_circles_pub, node);


        std::vector<double> loop_times; // Vector to store loop execution times

        // Measure loop execution time
        auto loop_end_time   = std::chrono::steady_clock::now();
        double loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time).count();
        loop_times.push_back(loop_duration);

        std_msgs::msg::Float64 loop_time_msg;
        loop_time_msg.data = loop_duration;
        loop_time_publisher->publish(loop_time_msg);
        // std::cout << "" << std::endl;
        // std::cout << "Loop execution time: " << loop_duration << " ms" << std::endl;
        //             std::cout << "" << std::endl;


        // Calculate and print average execution time
        static double total_loop_time = 0.0;
        static int loop_count         = 0;
        static int above_50_count     = 0;
        static double total_above_50  = 0.0;

        total_loop_time += loop_duration;
        loop_count++;

        if (loop_duration > 50.0) {
            above_50_count++;
            total_above_50 += (loop_duration - 50.0);
        }

        if (loop_count % 100 == 0) { // Print every 100 iterations
            double average_time = total_loop_time / loop_count;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "Average loop execution time: " << average_time << " ms" << std::endl;
            std::cout << "Number of loops above 50 ms: " << above_50_count << std::endl;
            std::cout << "Total time above 50 ms: " << total_above_50 << " ms" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
        }


        loop_rate.sleep();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();

    return 0;
}
