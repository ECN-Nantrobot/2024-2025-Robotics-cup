#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


class LaserToPointCloudNode : public rclcpp::Node
{
public:
    LaserToPointCloudNode() : Node("laser_to_pointcloud_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserToPointCloudNode::scanCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        const std::string target_frame = "map";

        sensor_msgs::msg::PointCloud2 cloud;
       
        try {
            projector_.transformLaserScanToPointCloud("lidar_link", *scan_msg, cloud, tf_buffer_);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "LasertoPointcloud Transform failed !!: %s", ex.what());
        }

        // Transform the point cloud from "lidar_link" frame to "map" frame
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(target_frame, scan_msg->header.frame_id,
                                                                                        tf2::TimePointZero, // Use the most recent available transform
                                                                                        tf2::durationFromSec(0.3));
        
            tf2::doTransform(cloud, transformed_cloud, transform);
            transformed_cloud.header.frame_id = target_frame; // Update the frame ID to "map"
            cloud_pub_->publish(transformed_cloud);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "PointCloud to map transform failed: %s", ex.what());
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}