#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DiffDrivePublisher : public rclcpp::Node
{
public:
    DiffDrivePublisher() : Node("diff_drive_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_base_controller/cmd_vel", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&DiffDrivePublisher::publishCommand, this));
    }

private:
    void publishCommand()
    {
        geometry_msgs::msg::Twist command;
        command.linear.x = 0.1;
        command.angular.z = 0.4;
        RCLCPP_INFO(this->get_logger(), "Publishing: Linear=%.2f, Angular=%.2f", command.linear.x, command.angular.z);
        publisher_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDrivePublisher>());
    rclcpp::shutdown();
    return 0;
}
