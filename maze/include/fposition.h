#ifndef FPOSITION
#define FPOSITION

#include <cstdlib>
#include <iostream>
// #include <math.h>
// #include <opencv2/opencv.hpp> // Include OpenCV
#include <algorithm> // Include algorithm for std::min and std::max

// namespace ecn {

// class Maze;

class FPosition {
public:

    FPosition (float _x=0.0, float _y=0.0): x(_x), y(_y) {}

    void operator=(const FPosition& p)
    {
        x = p.x;
        y = p.y;
    }




    // // prints the grid with all positions from parent
    // virtual void print(const Point &parent) const;  // Declaration only
    
    // void start();
    
    // // online print, color depends on closed / open set
    // virtual void show(bool closed, const Point &parent);
    
    // void drawCircle(cv::Mat &image, const cv::Scalar &color, int radius = 5) const;

    friend std::ostream& operator<<(std::ostream& out, const FPosition& p)
    {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

    // 2 positions are equal if they have the same x and y
    bool operator==(const FPosition& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const FPosition& other) const {
        return x != other.x || y != other.y;
    }

    // float heuristic(const Point &goal, bool use_manhattan) const
    // {
    //     if (use_manhattan){
    //         return std::abs(goal.x - x) + std::abs(goal.y - y);
    //     }
    //     else{
    //         return std::sqrt(std::pow(goal.x - x, 2) + std::pow(goal.y - y, 2)); // Euclidean distance
    //     }
    // }


    float x, y;
    // static Maze maze;
};

// }

#endif // POINT_H