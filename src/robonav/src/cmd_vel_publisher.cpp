// #include "geometry_msgs/msg/twist.hpp"
// #include "rclcpp/rclcpp.hpp"

// class CmdVelPublisher : public rclcpp::Node
// {
// public:
//     CmdVelPublisher() : Node("cmd_vel_publisher")
//     {
//         publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         timer_     = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdVelPublisher::publish_velocity, this));
//     }

// private:
//     void publish_velocity()
//     {
//         auto msg      = geometry_msgs::msg::Twist();
//         msg.linear.x  = 0.2;
//         msg.angular.z = 0.2;
//         publisher_->publish(msg);
//         RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear=%.2f angular=%.2f", msg.linear.x, msg.angular.z);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CmdVelPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }
