#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node      = rclcpp::Node::make_shared("test_cmd_vel_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    rclcpp::Rate loop_rate(1); // 1 Hz

    while (rclcpp::ok()) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x  = 0.1;
        msg.angular.z = 0.4;

        RCLCPP_INFO(node->get_logger(), "Publishing: Linear=%.2f, Angular=%.2f", msg.linear.x, msg.angular.z);
        publisher->publish(msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
