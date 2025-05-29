// #include <a_star.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
// #include <elastic_bands.h>
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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.hpp> // Provides pcl_ros::transformPointCloud

using namespace std;
using namespace ecn;

// enum RobotState { WAITING, INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };
enum RobotState { WAITING, INIT, PATH_PLANNING, NAVIGATION, GOAL_REACHED };

RobotState state = INIT;

Robot robot(Point::maze, 0, 0, 0, 18.5, 8, 25, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D

double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;

bool team_colour = 0;

ecn::Point pop;

double close_obstacle_threshold = 0.30;
bool emergencystate             = false;

int sendpathcounter = 0;


void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
                       Maze& maze,
                       float robot_pos_x,
                       float robot_pos_y,
                       const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& cloud_publisher,
                       const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& cloud_notused_publisher,
                       const rclcpp::Node::SharedPtr& node)
{
    robot_pos_x               = robot_x / 100.0;
    robot_pos_y               = 2 - (robot_y / 100.0);
    float reduction           = 0.25;
    float min_x               = 0 + reduction;
    float max_x               = 3 - reduction;
    float min_y               = 0 + reduction;
    float max_y               = 2 - reduction;
    float radius_around_robot = 0.3;

    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    int count_close_points = 0;

    // Filter points within the desired map area and exclude points within a radius of 30cm from the robot
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_notused(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& pt : cloud->points) {
        float distance_to_robot = std::sqrt(std::pow(pt.x - robot_pos_x, 2) + std::pow(pt.y - robot_pos_y, 2));
        if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y && distance_to_robot > radius_around_robot) {
            pcl::PointXYZRGB colored_pt;
            colored_pt.x = pt.x;
            colored_pt.y = pt.y;
            colored_pt.z = pt.z;
            colored_pt.r = 255;
            colored_pt.g = 255;
            colored_pt.b = 255;
            cloud_filtered->points.push_back(colored_pt);
        } else {
            pcl::PointXYZRGB colored_pt_notused;
            colored_pt_notused.x = pt.x;
            colored_pt_notused.y = pt.y;
            colored_pt_notused.z = pt.z;
            colored_pt_notused.r = 50;
            colored_pt_notused.g = 50;
            colored_pt_notused.b = 50;
            cloud_filtered_notused->points.push_back(colored_pt_notused);
        }

        if (distance_to_robot <= close_obstacle_threshold) {
            count_close_points++;
        }
    }


    emergencystate = false;

    if (count_close_points > 0) {
        emergencystate = true;
        std::cout << "⚠️ Emergency state activated! Close points detected: " << count_close_points << std::endl;
    } else {
        emergencystate = false;
    }

    // Perform clustering
    maze.resetIm(); // Reset the maze before processing new points

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.1); // minimum distance of points in a cluster
    ec.setMinClusterSize(3);     // Minimum number of points in a cluster
    ec.setMaxClusterSize(80);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    maze.resetIm(); // Reset the maze before processing new clusters

    // Map to store smoothed centers for each cluster
    static std::unordered_map<int, std::pair<float, float>> smoothed_centers;
    static bool first_time = true;
    const float alpha      = 0.85; // Smoothing factor (0 < alpha < 1) higher is less smoothing

    // Process each cluster
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        float sum_x = 0.0, sum_y = 0.0;
        for (const auto& idx : indices.indices) {
            const auto& pt = cloud_filtered->points[idx];
            sum_x += pt.x;
            sum_y += pt.y;
        }

        float cluster_center_x = sum_x / indices.indices.size();
        float cluster_center_y = sum_y / indices.indices.size();

        // Smooth the cluster center
        if (first_time) {
            smoothed_centers[cluster_id] = { cluster_center_x, cluster_center_y };
        } else {
            auto& smoothed  = smoothed_centers[cluster_id];
            smoothed.first  = alpha * cluster_center_x + (1 - alpha) * smoothed.first;
            smoothed.second = alpha * cluster_center_y + (1 - alpha) * smoothed.second;
        }

        const auto& smoothed = smoothed_centers[cluster_id];
        // std::cout << "Cluster " << cluster_id << " smoothed center: (" << smoothed.first << ", " << smoothed.second << ")" << std::endl;

        // Convert smoothed center to maze indices
        int maze_x = static_cast<int>(smoothed.first * 100);        // Convert meters to cm
        int maze_y = static_cast<int>(200 - smoothed.second * 100); // Convert meters to cm and adjust frame

        // Check bounds and color the maze black within a circular radius
        int radius = 30; // Define the radius around the pixel
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                if (dx * dx + dy * dy <= radius * radius) { // Check if the point is within the circle
                    int nx = maze_x + dx;
                    int ny = maze_y + dy;

                    // Ensure the new pixel is within bounds
                    if (nx >= 0 && nx < maze.im.cols && ny >= 0 && ny < maze.im.rows) {
                        maze.setPixel(nx, ny, 0); // Set pixel to black
                    }
                }
            }
        }

        cluster_id++;
    }

    first_time = false; // Set first_time to false after the first iteration


    sensor_msgs::msg::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    cloud_filtered_msg.header = cloud_msg->header;
    cloud_publisher->publish(cloud_filtered_msg);

    sensor_msgs::msg::PointCloud2 cloud_filtered_notused_msg;
    pcl::toROSMsg(*cloud_filtered_notused, cloud_filtered_notused_msg);
    cloud_filtered_notused_msg.header = cloud_msg->header;
    cloud_notused_publisher->publish(cloud_filtered_notused_msg);
}


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
    odom_msg.header.stamp    = node->get_clock()->now();
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
    size_t path_size       = std::min(path.size(), static_cast<size_t>(200)); // total points to save

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
    } else if (command.rfind("RESET!", 0) == 0) {
        // std::cout << "Received RESET command from ESP. Resetting robot state." << std::endl;
        // pid_t ppid = getppid();
        // std::cout << "Sending SIGINT to parent process (PID " << ppid << ")" << std::endl;
        // kill(ppid, SIGINT);

        // // Optionally, shutdown this node as well
        // rclcpp::shutdown();
        // return 1;
    }
}

std::vector<ecn::Point> generateStraightPath(const ecn::Point& start, const ecn::Point& end)
{
    std::vector<ecn::Point> path;

    float step_size = 1.5;

    float dx       = end.x - start.x;
    float dy       = end.y - start.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    int steps      = static_cast<int>(distance / step_size);

    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        float x = start.x + t * dx;
        float y = start.y + t * dy;

        ecn::Point point;
        point.x      = x;
        point.y      = y;
        point.radius = 3.0;                     // Optional default value
        point.colour = cv::Scalar(50, 200, 50); // Green-ish path point

        path.push_back(point);
    }

    return path;
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

// Function to load obstacles from a YAML file
void loadObstaclesFromFile(const std::string& filename, std::vector<Obstacle>& obstacles)
{
    YAML::Node yaml_config = YAML::LoadFile(filename);

    obstacles.clear();
    for (const auto& obstacle : yaml_config["obstacles"]) {
        int x             = obstacle["x"].as<int>();
        int y             = obstacle["y"].as<int>();
        int width         = obstacle["width"].as<int>();
        int height        = obstacle["height"].as<int>();
        std::string type  = obstacle["type"].as<std::string>();
        std::string color = obstacle["color"].as<std::string>();
        int duration      = 50000;

        Obstacle::Type obstacle_type = (type == "FIXED") ? Obstacle::FIXED : Obstacle::TEMPORARY;
        obstacles.emplace_back(x, y, width, height, obstacle_type, color, duration);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main_publisher");

    auto cloud_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_used", 10);

    auto cloud_notused_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_notused", 10);

    auto cloud_subscriber =
    node->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", 10, [node, cloud_publisher, cloud_notused_publisher](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        processPointCloud(msg, Point::maze, robot_x, robot_y, cloud_publisher, cloud_notused_publisher, node);
    });

    auto main_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/main", 10);
    auto path_publisher = node->create_publisher<nav_msgs::msg::Path>("elastic_band_path", 10);
    auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    // auto odom_subscriber    = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odomCallback);
    auto point_publisher    = node->create_publisher<geometry_msgs::msg::PointStamped>("point_topic", 10);
    auto occupancy_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_viz", 10);
    auto tf_broadcaster     = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto elastic_band_path_pub = node->create_publisher<nav_msgs::msg::Path>("/elastic_band_path", 10);
    // auto astar_path_pub           = node->create_publisher<nav_msgs::msg::Path>("/astar_path", 10);
    // auto elastic_band_circles_pub = node->create_publisher<visualization_msgs::msg::Marker>("/elastic_band_circles", 10);
    // auto loop_time_publisher      = node->create_publisher<std_msgs::msg::Float64>("loop_execution_time", 10);


    serial::Serial ser;
    bool port_opened = false;

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(921600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        if (ser.isOpen()) {
            std::cout << "Serial port opened successfully" << std::endl;
            port_opened = true;
        }
    } catch (serial::IOException& e) {
        std::cerr << "Unable to open ESP serial port: " << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give ESP time to start


    // Wait for the START and COLOUR! command from ESP
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() >= 1) {
            std::cout << "Waiting for START!" << std::endl;
            start_time = current_time;
        }
        if (ser.available()) {
            std::string command = ser.readline();
            std::cout << "Received from serial: " << command << std::endl;

            if (command.rfind("START!", 0) == 0) {
                command.erase(std::remove(command.begin(), command.end(), '\n'), command.end()); // Remove newline
                std::cout << "Received from serial: " << command << std::endl;
                std::cout << "Received START! command from ESP. Continuing..." << std::endl;

                while (ser.available()) {
                    std::string colour_command = ser.readline();
                    if (colour_command.rfind("COLOUR:", 0) == 0) {
                        std::string colour = colour_command.substr(7);                               // Extract the colour name
                        colour.erase(std::remove(colour.begin(), colour.end(), '\n'), colour.end()); // Remove newline
                        std::cout << "Received COLOUR:_" << colour << "_" << std::endl;
                        if (colour == "yellow") {
                            team_colour = 0;
                        } else if (colour == "blue") {
                            team_colour = 1;
                        } else {
                            std::cerr << "Unknown colour: " << colour << std::endl;
                        }
                        break;
                    }
                }
                break; // ✅ Break outer loop (START!)
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //-------------------------------------------------------------------------------------

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_wo_p.png");

    Point::maze.load(filename_maze);

    double wheel_distance, speed, kp, ki, kd;

    try {
        if (team_colour == 1) {
            loadGoalsFromFile("/home/pi/2024-2025-Robotics-cup/src/robonav/config/goals_blue_final.yaml", robot.goals, wheel_distance, speed, kp, ki, kd);
            std::cout << "Using blue goals configuration" << std::endl;
        } else if (team_colour == 0) {
            loadGoalsFromFile("/home/pi/2024-2025-Robotics-cup/src/robonav/config/goals_yellow_final.yaml", robot.goals, wheel_distance, speed, kp, ki, kd);
            std::cout << "Using yellow goals configuration" << std::endl;
        }
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

    // std::vector<Position> astar_path;
    // std::vector<Position> astar_path_previous;
    // ElasticBand elastic_band(astar_path, Point::maze);
    // elastic_band.resetOptimization(); // Setzt die Optimierung zurück

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


    // // Load obstacles from a YAML file
    // std::string obstacles_filename = "/home/pi/2024-2025-Robotics-cup/src/robonav/config/obstacles.yaml";
    // std::vector<Obstacle> obstacles;
    // try {
    //     loadObstaclesFromFile(obstacles_filename, obstacles);
    //     std::cout << "Obstacles loaded successfully from " << obstacles_filename << std::endl;
    // } catch (const std::exception& e) {
    //     std::cerr << "Error loading obstacles from yaml file: " << e.what() << std::endl;
    //     return 1;
    // }


    // cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);

    // try {
    //     astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);
    // } catch (const std::exception& e) {
    //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
    //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
    //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
    //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
    //     astar_path.clear(); // Clear the path to handle the error gracefully
    // }

    // for (auto& position : astar_path) {
    //     position           = position * (1 / Point::maze.resize_for_astar);
    //     astar_path.front() = start_p;
    //     astar_path.back()  = goal_p;
    // }


    // elastic_band.updatePath(astar_path);
    // elastic_band.runFullOptimization(robot.getPosition(), robot.goals[robot.goal_index]);


    // std::cout << "Waiting for ESP reset ... " << std::endl;
    // while (true) {
    //     if (ser.available()) {
    //         std::string result = ser.readline();
    //         if (result.find("ESP Initialized!") != std::string::npos) {
    //             std::cout << "ESP Initialization confirmed!" << std::endl;
    //             break;
    //         }
    //     }
    // }
    // std::cout << std::endl; // Move to the next line after the loop ends


    // publishElasticbandPath(elastic_band.getSmoothedPath(), path_publisher, node);
    publishOccupancyGrid(occupancy_grid_pub, Point::maze.im);

    sendSpeedAndPID(ser, speed, kp, ki, kd);
    sendGoals(ser);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // sendPath(ser, elastic_band.getSmoothedPath());

    auto straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
    sendPath(ser, straight_path);
    publishElasticbandPath(straight_path, path_publisher, node);


    std::this_thread::sleep_for(std::chrono::milliseconds(50000));

    ser.write("STATE:INIT\n");
    ser.write("STATE:INIT\n");
    ser.write("STATE:INIT\n");
    ser.write("STATE:INIT\n");
    ser.write("STATE:INIT\n");
    ser.write("STATE:INIT\n");
    std::cout << "Sent INIT state" << std::endl;

    std::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;

    // int navigation_counter = 0;
    // int send_path_counter = 0;

    rclcpp::Rate loop_rate(20); // 20 Hz, 50ms per loop


    bool dont_move = false;

    bool switch_sim_movoment = false;

    ////////////////////////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////////////////////
    while (rclcpp::ok()) {
        auto loop_start_time = std::chrono::steady_clock::now(); // Initialize loop start time

        if (dont_move == false) {


            int max_reads = 15;

            while (ser.available() && max_reads > 0) {
                std::string command = ser.readline();
                processCommand(command);
                max_reads--;
            }

            // // Updating the Obstacles
            // for (auto& obstacle : obstacles) {
            //     obstacle.update();
            // // }
            // Point::maze.renderObstacles(obstacles, Point::maze.im, 1);


            robot.setPose(robot_x, robot_y, robot_theta);

            switch (state) {
            case WAITING: std::cout << "State: WAITING" << std::endl; break;

            case INIT:
                std::cout << "State: INIT" << std::endl;

                // cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
                start_p = Position(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
                goal_p  = Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));
                // try {
                //     astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);
                // } catch (const std::exception& e) {
                //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
                //     astar_path.clear(); // Clear the path to handle the error gracefully
                //     // ser.write("STATE:WAITFORPATH\n");
                //     // std::cout << "Sent WAITFORPATH state" << std::endl;
                //     break;
                // }
                // for (auto& position : astar_path) {
                //     position           = position * (1 / Point::maze.resize_for_astar);
                //     astar_path.front() = start_p;
                //     astar_path.back()  = goal_p;
                // }

                


                // elastic_band.updatePath(astar_path);
                // elastic_band.runFullOptimization(robot.getPosition(), robot.goals[robot.goal_index]);

                // publishElasticbandPath(elastic_band.getSmoothedPath(), path_publisher, node);
                // sendPath(ser, elastic_band.getSmoothedPath());

                straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
                sendPath(ser, straight_path);
                publishElasticbandPath(straight_path, path_publisher, node);


                state = WAITING;


                break;

            case PATH_PLANNING:
                std::cout << "State: PATH_PLANNING-------" << std::endl;

                // Recalculate A* path
                // cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
                start_p = Position(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
                goal_p  = Position(static_cast<int>(robot.goals[robot.goal_index].point.x), static_cast<int>(robot.goals[robot.goal_index].point.y));

                // try {
                //     astar_path = Astar(start_p * Point::maze.resize_for_astar, goal_p * Point::maze.resize_for_astar);
                // } catch (const std::exception& e) {
                //     std::cerr << "Error during A* path calculation: " << e.what() << std::endl;
                //     astar_path.clear(); // Clear the path to handle the error gracefully
                //     ser.write("STATE:WAITFORPATH\n");
                //     std::cout << "Sent WAITFORPATH state" << std::endl;
                //     break;
                // }

                // for (auto& position : astar_path) {
                //     position           = position * (1 / Point::maze.resize_for_astar);
                //     astar_path.front() = start_p;
                //     astar_path.back()  = goal_p;
                // }

                // // Check the difference in the A* path
                // path_difference_front = 0.0;
                // path_difference_end   = 0.0;
                // path_difference       = 0.0;
                // // path_difference_check_limit = astar_path.size() < 25 ? astar_path.size() : astar_path.size() / 2;
                // path_difference_check_limit = astar_path.size() / 2;
                // if (astar_path_previous.size() != 0) {
                //     for (size_t i = 1 + path_difference_check_limit; i < astar_path.size(); i += 2) {
                //         double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                //         double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                //         path_difference_front += sqrt(dx * dx + dy * dy);
                //     }
                //     path_difference_front /= astar_path.size();
                //     // std ::cout << "A* path diff front: %: " << path_difference_front;

                //     for (size_t i = 1; i < astar_path.size() - path_difference_check_limit; i += 2) {
                //         double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                //         double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                //         path_difference_end += sqrt(dx * dx + dy * dy);
                //     }
                //     path_difference_end /= astar_path.size();
                //     // std ::cout << " A* path diff end: %: " << path_difference_end;
                // } else {
                //     // std::cout << "astar_path_previous.size() = " << astar_path_previous.size() << std::endl;
                // }
                // path_difference = path_difference_front + path_difference_end;
                // if (path_difference_front > 3) {
                //     astar_path_previous = astar_path;
                //     elastic_band.updatePath(astar_path);
                //     // std::cout << "    -> A* Path Front, set eb" << std::endl;
                //     elastic_band.resetOptimization();
                //     counter_set_eb_path       = 0;
                //     set_eb_path_counter_limit = 1;
                //     eb_comp_inarow            = 1;
                // } else if (path_difference_end > 4) {
                //     astar_path_previous = astar_path;
                //     elastic_band.updatePath(astar_path);
                //     // std::cout << "    -> A* Path Back, set eb" << std::endl;
                //     set_eb_path_counter_limit = 2;
                //     counter_set_eb_path       = 0;
                //     elastic_band.resetOptimization();
                // } else if (path_difference >= 3.1) {
                //     astar_path_previous = astar_path;
                //     elastic_band.updatePath(astar_path);
                //     // std::cout << "    -> A* Path FULL, set eb" << std::endl;
                //     elastic_band.resetOptimization();
                //     counter_set_eb_path       = 0;
                //     set_eb_path_counter_limit = 1;
                //     eb_comp_inarow            = 1;
                //     elastic_band.setMaxInterations(6);
                // } else if (path_difference > 2.6 && path_difference < 3 || astar_path_previous.size() == 0) {
                //     astar_path_previous = astar_path;
                //     elastic_band.updatePath(astar_path);
                //     // std::cout << "    -> A* Path MEDIUM, set eb" << std::endl;
                //     set_eb_path_counter_limit = 3;
                //     counter_set_eb_path       = 0;
                //     elastic_band.resetOptimization();
                // } else {
                //     std ::cout << "" << std::endl;
                // }


                // static auto last_clone_time = std::chrono::steady_clock::now();
                // auto current_time           = std::chrono::steady_clock::now();
                // if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() >= 5) {

                // sendpathcounter++;

                // if (sendpathcounter >= 50)
                // {
                //     straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
                //     sendPath(ser, straight_path);
                //     publishElasticbandPath(straight_path, path_publisher, node);

                //     std::cout << "Straight path sent and published!" << std::endl;
                //     std::cout << "Straight path sent and published!" << std::endl;
                //     std::cout << "Straight path sent and published!" << std::endl;
                //     std::cout << "Straight path sent and published!" << std::endl;
                //     std::cout << "Straight path sent and published!" << std::endl;

                //     sendpathcounter = 0;
                // }


                // elastic_band.updatePath(astar_path);


                // state = NAVIGATION;
                // [[fallthrough]];
                break;


            case NAVIGATION:
                std::cout << "State: NAVIGATION to: (" << robot.goals[robot.goal_index].point.x << ", " << robot.goals[robot.goal_index].point.y << ")" << std::endl;


                // elastic_band.optimize(robot.getPosition(), robot.goals[robot.goal_index]);

                // if (elastic_band.errorCheck() == true) {
                //     // std::cout << "EB Optimization error -> PATH PLANNING." << std::endl;
                //     elastic_band.resetOptimization();
                //     state = PATH_PLANNING;
                //     break;
                // }

                // if (elastic_band.isOptimizationComplete()) {
                //     // std::cout << "Elastic Band optimization completed";
                //     counter_set_eb_path++;
                //     elastic_band.resetOptimization();

                //     if (counter_set_eb_path > set_eb_path_counter_limit) {
                //         elastic_band.generateSmoothedPath(2.5f, 21, 1.2f); // 0.08 ms
                //         std::cout << "generated EB Path" << std::endl;

                //         set_eb_path_counter_limit = set_eb_path_counter_limit_default;
                //         eb_comp_inarow            = eb_comp_inarow_default;

                // send_path_counter++;
                // if (send_path_counter >= 5) {
                //     straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
                //     publishElasticbandPath(straight_path, path_publisher, node);
                //     sendPath(ser, straight_path);
                //     send_path_counter = 0;
                // }


                // navigation_counter++;

                // if (navigation_counter >= 15) {
                //     state              = PATH_PLANNING;
                //     navigation_counter = 0;
                // }

                break;

            case GOAL_REACHED:
                std::cout << "State: GOAL_REACHED" << std::endl;

                if (robot.goal_index < robot.goals.size() - 1) {
                    robot.goal_index++;
                    std::cout << "New goal: (" << robot.goals[robot.goal_index].point.x << ", " << robot.goals[robot.goal_index].point.y << ")" << std::endl;
                    // Remove the second obstacle from the obstacles vector
                    // int removeobstacles_offset = 0;
                    // if (team_colour == 1) { removeobstacles_offset = 2; }
                    // if (obstacles.size() > 1 + removeobstacles_offset) {
                    //     obstacles.erase(obstacles.begin());
                    //     std::cout << "OOOOOOOOOOOOO Removed first obstacle from obstacles vector OOOOOOOOOOOOO" << std::endl;
                    // }


                    straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
                    sendPath(ser, straight_path);
                    publishElasticbandPath(straight_path, path_publisher, node);

                } else {
                    std::cout << "All goals reached!" << std::endl;
                }

                state = WAITING;


                break;
            }
        }

        static auto last_clone_time = std::chrono::steady_clock::now();
        auto current_time           = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() << std::endl;
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() >= 1)
        {
            last_clone_time = current_time;
            publishOccupancyGrid(occupancy_grid_pub, Point::maze.im);
            std::cout << "Occupancy grid published!!!!" << std::endl;
        }
        // if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() >= 5) {

        //     straight_path = generateStraightPath(robot.getPosition(), robot.goals[robot.goal_index]);
        //     sendPath(ser, straight_path);
        //     publishElasticbandPath(straight_path, path_publisher, node);

        //     std::cout << "Straight path sent and published!" << std::endl;
        //     std::cout << "Straight path sent and published!" << std::endl;
        //     std::cout << "Straight path sent and published!" << std::endl;
        //     std::cout << "Straight path sent and published!" << std::endl;
        //     std::cout << "Straight path sent and published!" << std::endl;

        // }

        rclcpp::spin_some(node);

        publishPosition(odom_publisher, tf_broadcaster, node);
        publishPoint(point_publisher, node, pop);
        // publishAStarPath(astar_path, astar_path_pub, node);
        // publishElasticBandCircles(elastic_band.getSmoothedPath(), elastic_band_circles_pub, node);


        std::vector<double> loop_times; // Vector to store loop execution times
        // Measure loop execution time
        auto loop_end_time   = std::chrono::steady_clock::now();
        double loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time).count();
        loop_times.push_back(loop_duration);

        std_msgs::msg::Float64 loop_time_msg;
        loop_time_msg.data = loop_duration;
        // loop_time_publisher->publish(loop_time_msg);

        std::cout << "Loop execution time: " << loop_duration << " ms" << std::endl;

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
            std::cout << "Average loop execution time: " << average_time << " ms" << std::endl;
            std::cout << "Number of loops above 50 ms: " << above_50_count << std::endl;
            std::cout << "Total time above 50 ms: " << total_above_50 << " ms" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "" << std::endl;
        }

        if (emergencystate == true) {
            ser.write("EMERGENCYSTOP\n");
            std::cout << "send emergencystopppppppppppppp" << std::endl;
        }


        loop_rate.sleep();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();

    return 0;
}
