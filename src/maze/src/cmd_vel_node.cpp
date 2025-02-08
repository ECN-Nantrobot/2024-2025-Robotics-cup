#include "elastic_bands.h"
#include "robot.h"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
    : Node("cmd_vel_publisher"), robot(ecn::Maze(), 50, 50, 0, 15, 7, 10, 0.01, 0.5), elastic_band(std::vector<ecn::Point>(), ecn::Maze()) // Explizite Initialisierung
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_     = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdVelPublisher::publish_velocity, this));
    }

private:
    void publish_velocity()
    {
        auto msg      = geometry_msgs::msg::Twist();
        msg.linear.x  = (robot.getLeftSpeed() + robot.getRightSpeed()) / 2.0;
        msg.angular.z = (robot.getRightSpeed() - robot.getLeftSpeed()) / robot.getWheelBase();

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    ecn::Robot robot;
    ecn::ElasticBand elastic_band;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
