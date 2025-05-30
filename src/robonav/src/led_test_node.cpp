#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <gpiod.h>
#include <memory>

using namespace std::chrono_literals;

class LEDNode : public rclcpp::Node
{
public:
    LEDNode() : Node("led_node"), led_on_(false)
    {
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open gpiochip0");
            rclcpp::shutdown();
            return;
        }

        line_ = gpiod_chip_get_line(chip_, 16);
        if (!line_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get line 16");
            gpiod_chip_close(chip_);
            rclcpp::shutdown();
            return;
        }

        if (gpiod_line_request_output(line_, "led_node", 0) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to request line as output");
            gpiod_chip_close(chip_);
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(1s, std::bind(&LEDNode::toggle_led, this));
        RCLCPP_INFO(this->get_logger(), "LED node started (GPIO 16)");
    }

    ~LEDNode()
    {
        gpiod_line_release(line_);
        gpiod_chip_close(chip_);
    }

private:
    void toggle_led()
    {
        led_on_ = !led_on_;
        gpiod_line_set_value(line_, led_on_);
        RCLCPP_INFO(this->get_logger(), "LED %s", led_on_ ? "ON" : "OFF");
    }

    struct gpiod_chip* chip_;
    struct gpiod_line* line_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool led_on_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LEDNode>());
    rclcpp::shutdown();
    return 0;
}
