#ifndef POINT_H
#define POINT_H

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
// #include <maze.h>

namespace ecn
{

class Maze;

class Point
{
public:
    Point(float _x = 0, float _y = 0, float _radius = 6, cv::Scalar _colour = cv::Scalar(0, 0, 255)) : x(_x), y(_y), radius(_radius), colour(_colour) {}

    void operator=(const Point& p)
    {
        x = p.x;
        y = p.y;
    }

    Point toInt() const { return Point(static_cast<int>(x), static_cast<int>(y), radius); }
    Point toFloat() const { return Point(static_cast<float>(x), static_cast<float>(y), radius); }


    void start();

    // online print, color depends on closed / open set
    virtual void show(bool closed, const Point& parent);
    // prints the grid with all positions from parent
    virtual void print(const Point& parent) const; // Declaration only

    friend std::ostream& operator<<(std::ostream& out, const Point& p)
    {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

    bool operator==(const Point& other) const { return std::round(x) == std::round(other.x) && std::round(y) == std::round(other.y); }
    bool operator!=(const Point& other) const { return std::round(x) != std::round(other.x) || std::round(y) != std::round(other.y); }

    double heuristic(const Point& goal) const
    {
        return std::sqrt(std::pow(goal.x - x, 2) + std::pow(goal.y - y, 2)); // Euclidean distance
    }

    float x, y;
    float radius;
    cv::Scalar colour;
    static Maze maze;
    // static Maze *maze;
};

} // namespace ecn

#endif // POINT_H