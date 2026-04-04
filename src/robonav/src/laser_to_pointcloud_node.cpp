#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // for doTransform on PointCloud2

class LidarTransformNode : public rclcpp::Node
{
public:
    LidarTransformNode() : Node("lidar_transform_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LidarTransformNode::scanCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    laser_geometry::LaserProjection projector_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // 1. Project the scan to point cloud in lidar_link frame
        sensor_msgs::msg::PointCloud2 cloud_lidar;
        projector_.projectLaser(*scan_msg, cloud_lidar);

        cloud_lidar.header.frame_id = scan_msg->header.frame_id;
        cloud_lidar.header.stamp    = scan_msg->header.stamp;

        // 2. Lookup transform from lidar_link to map
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("map", scan_msg->header.frame_id, // from lidar_link to map
                                                   scan_msg->header.stamp, rclcpp::Duration::from_seconds(0.4));
        } catch (tf2::TransformException& ex) {
            // RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        // 3. Transform the point cloud
        sensor_msgs::msg::PointCloud2 cloud_map;
        tf2::doTransform(cloud_lidar, cloud_map, transform);
        cloud_map.header.frame_id = "map";

        // 4. Publish the transformed point cloud
        cloud_pub_->publish(cloud_map);
        // RCLCPP_INFO(this->get_logger(), "Published transformed point cloud to 'map' frame.");

     
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTransformNode>());
    rclcpp::shutdown();
    return 0;
}
