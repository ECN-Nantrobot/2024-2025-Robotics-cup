#ifndef LED_CONTROL_HPP_
#define LED_CONTROL_HPP_

#include <atomic>
#include <chrono>
#include <gpiod.h>
#include <stdexcept>
#include <string>
#include <thread>

class LEDControl
{
public:
    LEDControl(int gpio = 16, const std::string& chipname = "gpiochip0") : flashing_(false)
    {
        chip_ = gpiod_chip_open_by_name(chipname.c_str());
        if (!chip_) {
            throw std::runtime_error("Failed to open GPIO chip");
        }

        line_ = gpiod_chip_get_line(chip_, gpio);
        if (!line_) {
            gpiod_chip_close(chip_);
            throw std::runtime_error("Failed to get GPIO line");
        }

        if (gpiod_line_request_output(line_, "led_control", 0) < 0) {
            gpiod_chip_close(chip_);
            throw std::runtime_error("Failed to request line as output");
        }
    }

    ~LEDControl()
    {
        stop_flash();
        if (line_) {
            gpiod_line_set_value(line_, 0);
            gpiod_line_release(line_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }

    void turn_on() { gpiod_line_set_value(line_, 1); }

    void turn_off() { gpiod_line_set_value(line_, 0); }

    void start_flash_slow()
    {
        start_flash(1000); // 1s on/off
    }

    void start_flash_fast()
    {
        start_flash(200); // 200ms on/off
    }

    void start_long_on_flash()
    {
        start_flash(500); // 1s on/off
    }

    void stop_flash()
    {
        flashing_ = false;
        if (flash_thread_.joinable()) {
            flash_thread_.join();
        }
        turn_off();
    }

private:
    void start_flash(int interval_ms)
    {
        stop_flash(); // stop any existing thread
        flashing_     = true;
        flash_thread_ = std::thread([this, interval_ms]() {
            while (flashing_) {
                turn_on();
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
                turn_off();
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        });
    }

    void start_long_on_flash(int interval_ms)
    {
        stop_flash(); // stop any existing thread
        flashing_     = true;
        flash_thread_ = std::thread([this, interval_ms]() {
            while (flashing_) {
                turn_on();
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
                turn_off();
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms/2));
            }
        });
    }


    struct gpiod_chip* chip_;
    struct gpiod_line* line_;
    std::thread flash_thread_;
    std::atomic<bool> flashing_;
};

#endif // LED_CONTROL_HPP_
