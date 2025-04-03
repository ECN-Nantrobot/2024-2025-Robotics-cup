#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

        if (!tf_buffer_.canTransform(target_frame, scan_msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5))) {
            RCLCPP_WARN(this->get_logger(), "Transform unavailable: %s → %s", scan_msg->header.frame_id.c_str(), target_frame.c_str());
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud;
        try {
            // ✅ Use current time instead of scan time
            sensor_msgs::msg::LaserScan scan_copy = *scan_msg;
            scan_copy.header.stamp                = this->now();

            projector_.transformLaserScanToPointCloud(target_frame, scan_copy, cloud, tf_buffer_);
            cloud_pub_->publish(cloud);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
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
