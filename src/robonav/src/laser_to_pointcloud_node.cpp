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

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_transformed", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        const std::string target_frame = "map";
        const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.3);


        sensor_msgs::msg::LaserScan transformed_scan_msg = *scan_msg;
        transformed_scan_msg.header.frame_id             = target_frame;
        scan_pub_->publish(transformed_scan_msg);


        // Check if the transform is available before calling projector
        if (!tf_buffer_.canTransform(target_frame, transformed_scan_msg.header.frame_id, transformed_scan_msg.header.stamp, timeout)) {
            RCLCPP_WARN(this->get_logger(), "Transform from %s to %s at time (%u.%u) not available yet. Skipping scan.", transformed_scan_msg.header.frame_id.c_str(),
                        target_frame.c_str(), transformed_scan_msg.header.stamp.sec, transformed_scan_msg.header.stamp.nanosec);
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud;
        try {
            // Transform and project laser scan into point cloud directly in target frame
            projector_.transformLaserScanToPointCloud(target_frame, transformed_scan_msg, cloud, tf_buffer_);

            cloud.header.frame_id = target_frame; // Ensure correct frame ID
            cloud_pub_->publish(cloud);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "LaserScan to PointCloud transform failed: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

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
