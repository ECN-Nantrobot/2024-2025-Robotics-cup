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

// #include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
using namespace ecn;


enum RobotState { INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };

void publishPath(const std::vector<Point>& elastic_path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher, const rclcpp::Node::SharedPtr& node)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp    = node->now();
    path_msg.header.frame_id = "map"; 

    for (const auto& point : elastic_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp       = path_msg.header.stamp;
        pose.header.frame_id    = "map";
        pose.pose.position.x    = point.x/100.0;  // Convert from cm to m
        pose.pose.position.y    = 2 - point.y/100.0; // Conversion cm to m and frame change
        pose.pose.position.z    = 0.0;
        pose.pose.orientation.w = 1.0; // Default orientation

        path_msg.poses.push_back(pose);
    }

    path_publisher->publish(path_msg);
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
    grid.header.frame_id           = "map";

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

void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, Maze& maze)
{
    float reduction = 0.15;
    float min_x     = 0 + reduction;
    float max_x     = 3 + reduction;
    float min_y     = 0 - reduction;
    float max_y     = 2 - reduction;

    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Filter points within the desired map area
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& pt : cloud->points) {
        if (pt.x >= min_x && pt.x <= max_x && pt.y >= min_y && pt.y <= max_y) {
            cloud_filtered->points.push_back(pt);
        }
    }

    // Perform clustering
    maze.resetIm(); // Reset the maze before processing new points

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08); // 5 cm distance of points
    ec.setMinClusterSize(3);      // Minimum number of points in a cluster
    ec.setMaxClusterSize(30);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    maze.resetIm(); // Reset the maze before processing new clusters

    // Calculate cluster center and average it with the previous cluster center
    for (const auto& indices : cluster_indices) {
        float sum_x = 0.0, sum_y = 0.0;
        for (const auto& idx : indices.indices) {
            const auto& pt = cloud_filtered->points[idx];
            sum_x += pt.x;
            sum_y += pt.y;
        }

        float cluster_center_x = sum_x / indices.indices.size();
        float cluster_center_y = sum_y / indices.indices.size();

        // // Average with the previous cluster center
        // static float prev_center_x = cluster_center_x;
        // static float prev_center_y = cluster_center_y;

        // float avg_center_x = (cluster_center_x + prev_center_x) / 2.0;
        // float avg_center_y = (cluster_center_y + prev_center_y) / 2.0;

        // prev_center_x = avg_center_x;
        // prev_center_y = avg_center_y;

        // Convert averaged center to maze indices
        int maze_x = static_cast<int>(cluster_center_x * 100);   // Convert meters to cm
        int maze_y = static_cast<int>(200 - cluster_center_y * 100); // Convert meters to cm and adjust frame

        // Check bounds and color the maze black within a circular radius
        int radius = 30; // Define the radius around the pixel
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
            if (dx * dx + dy * dy <= radius * radius) { // Check if the point is within the circle
                int nx = maze_x + dx;
                int ny = maze_y + dy;

                // Ensure the new pixel is within bounds
                if (nx >= 0 && nx < maze.im.rows && ny >= 0 && ny < maze.im.cols) {
                maze.setPixel(nx, ny, 0); // Set pixel to black
                }
            }
            }
        }
    }
}


double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;


void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Get `x` and `y`, convert from meters to cm
    robot_x = msg->pose.pose.position.x * 100;
    robot_y = 200 - msg->pose.pose.position.y * 100;

    // Correctly Convert Quaternion to Yaw (Theta)
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Extract yaw angle

    robot_theta = - yaw; // Now theta (yaw) is correct

    RCLCPP_INFO(rclcpp::get_logger("Odometry"), "Odom: x=%.2f, y=%.2f, theta=%.2f°", robot_x, robot_y, robot_theta * 180 / M_PI);
}

void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Get `x` and `y`, convert from meters to cm
    robot_x = msg->pose.pose.position.x * 100;
    robot_y = 200 - msg->pose.pose.position.y * 100;

    // Correctly Convert Quaternion to Yaw (Theta)
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Extract yaw angle

    robot_theta = -yaw; // Now theta (yaw) is correct

    RCLCPP_INFO(rclcpp::get_logger("AMCL"), "AMCL: x=%.2f, y=%.2f, theta=%.2f°", robot_x, robot_y, robot_theta * 180 / M_PI);
}

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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node               = rclcpp::Node::make_shared("main_node_publisher");
    auto main_node_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto path_publisher     = node->create_publisher<nav_msgs::msg::Path>("elastic_band_path", 10);
    // USE ODOM FOR POSITION
    auto odom_subscriber    = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odomCallback);
    // USE LIDAR AMCL FOR POSITION
    // auto amcl_subscriber    = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, amclCallback);
    auto occupancy_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map_viz", 10);
    auto cloud_subscriber =   node->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", 10, [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { processPointCloud(msg, Point::maze); });

    last_time = node->now();

    //-------------------------------------------------------------------------------------
    RobotState state = INIT;

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);
    Point::maze.computeDistanceTransform(); // Precompute distance transform

    Point start;
    Point goal;
    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));

    std::vector<Point> goals = { Point(270, 150), Point(40, 40), Point(35, 170), Point(220, 50)}; // goal points
    int current_goal_index   = 1;   // Keep track of which goal the robot is targeting
    start                    = goals[0];
    goal                    = goals[current_goal_index];
    std::vector<double> target_thetas(goals.size(), 0); // Initialize target thetas
    target_thetas = { 45 *M_PI/180, 0, 0, 0}; // Set target thetas for each goal

    int counter_set_eb_path = 0;

    Robot robot(Point::maze, start.x, start.y, target_thetas[0] * M_PI / 180, 33, 4, 14, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D
    robot.setIsStarting(true);  // Enable gradual start
    robot.setPose(goals[0].x, goals[0].y, target_thetas[0] * M_PI / 180);
    robot.setTargetTheta(target_thetas[0]);

    std::cout << "Robot TAAAAAQARGHETTTTTT Theta: " << robot.getTheta() * 180 / M_PI << "°" << std::endl;


    std::vector<Position> astar_path;
    std::vector<Position> astar_path_previous;
    ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.resetOptimization(); // Setzt die Optimierung zurück

    // const float dt = 0.001;
    int t          = 0;

    float distance_to_goal                = robot.distanceToGoal(goal);
    int set_eb_path_counter_limit_default = 1;
    int set_eb_path_counter_limit         = set_eb_path_counter_limit_default;
    int eb_comp_inarow_default            = 1;
    int eb_comp_inarow                    = eb_comp_inarow_default;

    std::vector<Obstacle> obstacles = {
        Obstacle(30, 100, 5, 5, Obstacle::MOVABLE, "lightgray", 0, 0, 0),
        // Obstacle(0, 20, 30, 15, Obstacle::MOVABLE, "lightgray", 0, 3 * dt, 1 * dt),
        // Obstacle(299, 25, 30, 15, Obstacle::MOVABLE, "lightgray", 0, -3 * dt, 2 * dt),
        // Obstacle(350, 80, 65, 10, Obstacle::MOVABLE, "lightgray", 0, -4 * dt, 1 * dt)
    };

    // Vizualize with OpenCV 1 of 2
    // const int scale       = 20; // size up visualization for better quality
    // const int display_res = 600;
    // cv::Mat simulation;
    // cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
    // cv::cvtColor(simulation, simulation, cv::COLOR_BGR2BGRA); // to support transparency
    // std::string window_name = ("Robot Simulation");
    // cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    // cv::moveWindow(window_name, 0, 0);
    // cv::resizeWindow(window_name, display_res * simulation.cols / simulation.rows, display_res * simulation.rows / simulation.rows);

    std::chrono::time_point<std::chrono::steady_clock> astarStartTime, astarEndTime, elasticStartTime, elasticEndTime, realStartTime, realEndTime, loopStartTime,
    loopEndTime; std::chrono::duration<double> astarDuration, elasticDuration, realDuration, loopDuration;
    float path_difference           = 0.0;
    float path_difference_front     = 0.0;
    float path_difference_end       = 0.0;
    int path_difference_check_limit = 0;



    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Wait for robot to be spawned
    RCLCPP_INFO(node->get_logger(), "Waiting for robot to be spawned in Gazebo...");

    while (rclcpp::ok()) {
        try {
            // Try to lookup transform from 'odom' to 'base_link' (i.e., check if the robot exists)
            auto transform = tf_buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
            RCLCPP_INFO(node->get_logger(), "Robot spawn confirmed! Starting loop...");
            break;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(node->get_logger(), "Robot not spawned yet... waiting.");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // Wait before retrying
    }
    rclcpp::sleep_for(std::chrono::seconds(2));


    realStartTime = std::chrono::steady_clock::now();

    ////////////////////////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////////////////////
    while (rclcpp::ok()) {
        std::cout << "Robot Theta: " << robot.getTheta() * 180 / M_PI << "°" << std::endl;

     
        double dt = getDt(node);
        //     loopStartTime = std::chrono::steady_clock::now();

        // Updating the Map
        for (auto& obstacle : obstacles)
        {
            obstacle.update();
        }
        Point::maze.renderObstacles(obstacles, Point::maze.im);


        switch (state) {
        case INIT:
            std::cout << "State: INIT, goal: (" << goals[current_goal_index].x << ", " << goals[current_goal_index].y << ")" << std::endl;

            robot.setIsStarting(true); // Enable gradual start

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
            // std::cout << "    -> A* Path Set" << std::endl;

            elastic_band.runFullOptimization(start, goal);
            // std::cout << "Elastic Band Path completed full optimization, -> Path Set" << std::endl;
            publishPath(elastic_band.getSmoothedPath(), path_publisher, node);
            
            state = TURN_TO_PATH;
            // state = PATH_PLANNING;
            [[fallthrough]];


        case TURN_TO_PATH:
            std::cout << "State: TURN_TO_PATH" << std::endl;

            if (robot.turnToPathOrientation(dt, elastic_band.getSmoothedPath())) {
                std::cout << "Robot is aligned to path orientation!" << std::endl;
                robot.setIsStarting(true);
                state = NAVIGATION;
            }

            break;


        case PATH_PLANNING:
            std::cout << "State: PLANNING" << std::endl;

            // astarStartTime = std::chrono::steady_clock::now();

            // Recalculate A* path
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


            // astarEndTime  = std::chrono::steadyStartTime;
            // std::cout << "A* completed in: " << std::fixed << std::setprecision(3) << astarDuration.count() * 1000 << " ms" << std::endl;

            state = NAVIGATION;
            [[fallthrough]];


        case NAVIGATION:
            std::cout << "State: NAVIGATION, goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

            // elasticStartTime = std::chrono::steady_clock::now();


            // for (int i = 0; i < eb_comp_inarow; i++) {
            //     Point::maze.computeDistanceTransform();
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
                    // std ::cout << "counter_set_eb_path > set_eb_path_counter_limit: " << counter_set_eb_path << " > " << set_eb_path_counter_limit << std::endl;
                    elastic_band.generateSmoothedPath(0.8f, 21, 1.2f); // 0.08 ms
                    // std::cout << "  -> Path Set" << std::endl;

                    // counter++;
                    // if(counter >= 2){
                    set_eb_path_counter_limit = set_eb_path_counter_limit_default;
                    eb_comp_inarow            = eb_comp_inarow_default;
                    // }

                    publishPath(elastic_band.getSmoothedPath(), path_publisher, node);


                } else {
                    // std::cout << "" << std::endl;
                }
            }

            // elasticEndTime  = std::chrono::steady_clock::now();
            // elasticDuration = elasticEndTime - elasticStartTime;
            // std::cout << "Elastic Band optimization completed in: " << std::fixed << std::setprecision(3) << elasticDuration.count() * 1000 << " ms" << std::endl;


            robot.followPath(elastic_band.getSmoothedPath(), Point::maze, dt);

            start = Point(robot.getX(), robot.getY());

            distance_to_goal = robot.distanceToGoal(goal);
            if (distance_to_goal < 5.0) {
                std::cout << "Distance to goal: " << distance_to_goal << std::endl;

                if (distance_to_goal < 0.5) {
                    std::cout << "Robot has reached the goal -> TURN_TO_GOAL!" << std::endl;
                    state = TURN_TO_GOAL;
                }

            } else if (static_cast<int>(realDuration.count()) % 1 == 0) {
                state = PATH_PLANNING;
            } else {
                state = NAVIGATION;
            }

            break;


        case TURN_TO_GOAL:
            std::cout << "State: TURN_TO_GOAL" << std::endl;

            if (robot.turnToGoalOrientation(dt)) {
                std::cout << "Robot is aligned to target orientation!" << std::endl;
                state = GOAL_REACHED;
            }

            break;


        case GOAL_REACHED:
            std::cout << "State: GOAL_REACHED: (" << goal.x << ", " << goal.y << ")" << std::endl;

            if (current_goal_index + 1 < goals.size()) {
                current_goal_index++;
                robot.setTargetTheta(target_thetas[current_goal_index]);
                state = INIT;
            } else {
                std::cout << "All goals have been reached. Mission complete!" << std::endl;
                return 0; // End simulation
            }
            break;
        }

        auto msg = geometry_msgs::msg::Twist();

        // Example: Use your robot's computed speed and rotation
        msg.linear.x  = robot.getLinearVelocity() /100;  // Replace with your velocity computation
        msg.angular.z = - robot.getAngularVelocity(); // Replace with your rotation computation
        // RCLCPP_INFO(node->get_logger(), "Publishing velocity: linear.x = %f, angular.z = %f", msg.linear.x, msg.angular.z);
        main_node_publisher->publish(msg);

        // static auto last_clone_time = std::chrono::steady_clock::now();
        // auto current_time           = std::chrono::steady_clock::now();
        // if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_clone_time).count() >= 0.5) {
        //     last_clone_time = current_time;
            publishOccupancyGrid(occupancy_grid_pub, Point::maze.im);
        // }

        rclcpp::spin_some(node); // Allow ROS 2 callbacks to be processed
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // robot.updatePosition(dt); // Update robot position with integration
        
        robot.setPose(robot_x, robot_y, robot_theta); // Update robot position with odometry from gazebo


        // Vizualize with OpenCV 2 of 2
        // cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
        // cv::cvtColor(simulation, simulation, cv::COLOR_BGR2BGRA);                                                                    // to support transparency
        // robot.draw(simulation, elastic_band.getSmoothedPath(), scale, elastic_band.getInitialPath(), goals, elastic_band.getPath(), astar_path); // takes 1ms
        // cv::imshow(window_name, simulation);
        // cv::waitKey(1);


        // loopEndTime  = std::chrono::steady_clock::now();
        // loopDuration = loopEndTime - loopStartTime;
        // if (loopDuration.count() * 1000 > dt * 1000) {
        //     // std::cout << "WARNING: Loop took longer than dt!!!!!!!!!! Time taken: " << loopDuration.count() * 1000 << " ms, Expected: " << dt * 1000 << " ms" <<
        //     std::endl; cv::waitKey(1);

        // } else {
        //     // std::cout << "Loop running in real-time. Time taken: " << loopDuration.count() * 1000 << " ms" << std::endl;
        //     cv::waitKey(static_cast<int>(std::ceil(dt * 1000 - loopDuration.count() * 1000)));
        // }

        // t++;
        // // std::cout << "Simulation time: " << std::fixed << std::setprecision(3) << t * dt << " s" << std::endl;
        realEndTime  = std::chrono::steady_clock::now();
        realDuration = realEndTime - realStartTime;
        std::cout << "Real Time:       " << std::fixed << std::setprecision(3) << realDuration.count() << " s" << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
