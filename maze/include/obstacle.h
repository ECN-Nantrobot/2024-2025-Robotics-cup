#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

class Obstacle
{
public:
    enum Type
    {
        FIXED,     // Fixed obstacle, always active
        TEMPORARY, // Temporary obstacle, deactivates after duration
        MOVABLE    // Movable obstacle, can be repositioned
    };

    Obstacle(int x, int y, int width, int height, Type type, const std::string& colorName, int duration = 0, int dx = 0, int dy = 0)
    : x_(x), y_(y), x_prev(x), y_prev(y), width_(width), height_(height), type_(type), duration_(duration), active_(true), dx_(dx), dy_(dy)
    {
        color_ = getColorFromName(colorName);
    }

    void update()
    {
        if (type_ == TEMPORARY && duration_ > 0)
        {
            --duration_;
            if (duration_ == 0)
            {
                active_ = false;
            }
        }
        if(type_ == MOVABLE && active_)
        {
                x_prev = x_;
                y_prev = y_;
                x_ += dx_;
                y_ += dy_;
        } 
    }

    void moveTo(int newX, int newY)
    {
        if (type_ == MOVABLE)
        {
            x_prev = x_;
            y_prev = y_;
            x_ = newX;
            y_ = newY;
        }
    }

    void remove()
    {
        if (type_ == TEMPORARY)
        {
            active_ = false;
        }
    }

    bool isActive() const { return active_; }
    int getX() const { return x_; }
    int getY() const { return y_; }
    int getXPrev() const { return x_prev; }
    int getYPrev() const { return y_prev; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    cv::Scalar getColor() const { return color_; }
    Type getType() const { return type_; }

private:
    int x_, y_;          // Position of the obstacle
    int x_prev, y_prev;    // Previous position
    int dx_, dy_;          // Movement increments for MOVABLE obstacles


    int width_, height_; // Dimensions of the obstacle
    Type type_;          // Type of the obstacle
    int duration_;       // Duration for TEMPORARY obstacles
    bool active_;        // Is the obstacle active?
    cv::Scalar color_;   // Color of the obstacle

    static cv::Scalar getColorFromName(const std::string& colorName)
    {
        if (colorName == "red")
        {
            return cv::Scalar(0, 0, 255);
        }
        else if (colorName == "blue")
        {
            return cv::Scalar(255, 0, 0);
        }
        else if (colorName == "green")
        {
            return cv::Scalar(0, 255, 0);
        }
        else if (colorName == "black")
        {
            return cv::Scalar(0, 0, 0);
        }
        else
        {
            throw std::invalid_argument("Unknown color: " + colorName);
        }
    }
};

#endif // OBSTACLE_H
