#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointCommandsPublisher : public rclcpp::Node
{
public:
    JointCommandsPublisher() : Node("joint_commands_publisher")
    {
        // Create the publisher for joint commands
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

        // Publish at 100ms intervals
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JointCommandsPublisher::publish_joint_commands, this));

        RCLCPP_INFO(this->get_logger(), "joint_commands_publisher node started.");
    }

private:
    void publish_joint_commands()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();

        // Specify the joints (adjust names to match your URDF/SDF)
        msg.name = { "left_wheel_joint", "right_wheel_joint" };

        // Set target velocities (adjust for movement control)
        msg.velocity = { 1.0, 1.0 }; // Move forward with equal speed

        RCLCPP_INFO(this->get_logger(), "Publishing /joint_commands: Left=%.2f, Right=%.2f", msg.velocity[0], msg.velocity[1]);

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointCommandsPublisher>());
    rclcpp::shutdown();
    return 0;
}
