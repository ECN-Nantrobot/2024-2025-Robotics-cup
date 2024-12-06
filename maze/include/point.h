#ifndef POINT_H
#define POINT_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
// #include <maze.h>

namespace ecn {

class Maze;

class Point {
public:

    Point(float _x=0, float _y=0): x(_x), y(_y){}

    void operator=(const Point &p){
        x = p.x;
        y = p.y;
    }

    Point toInt() const { return Point(static_cast<int>(x), static_cast<int>(y)); }
    Point toFloat() const { return Point(static_cast<float>(x), static_cast<float>(y)); }

    
    void start();
    
    // online print, color depends on closed / open set
    virtual void show(bool closed, const Point &parent);
    // prints the grid with all positions from parent
    virtual void print(const Point &parent) const;  // Declaration only

    // void drawCircle(cv::Mat &image, const cv::Scalar &color, int radius = 5) const;

    friend std::ostream& operator<<(std::ostream& out, const Point& p)
    {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

    // 2 positions are equal if they have the same x and y
    bool operator==(const Point &other) const{
        return std::round(x) == std::round(other.x) &&
               std::round(y) == std::round(other.y);
    }

    bool operator!=(const Point &other) const {
        return std::round(x) != std::round(other.x) || 
               std::round(y) != std::round(other.y);
    }

    double heuristic(const Point &goal, bool use_manhattan) const
    {
        if (use_manhattan){
            return std::abs(goal.x - x) + std::abs(goal.y - y);
        }
        else{
            return std::sqrt(std::pow(goal.x - x, 2) + std::pow(goal.y - y, 2)); // Euclidean distance
        }
    }

    float x, y;
    static Maze maze;
    // static Maze *maze;
};

}

#endif // POINT_H