#ifndef MAZE_H
#define MAZE_H

#include "point.h" // Include point.h to get the full definition of Point
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ecn
{

class Point; // Forward declaration of Point

typedef std::pair<int, int> Pair;

class Maze
{
public:
    static std::string mazeFile(const std::string& file = "maze.png");

    Maze();
    void load(std::string _filename);

    bool isFree(float fx, float fy) const;
    bool isFree(const Point& p) const;

    inline int height() const { return im.rows; }
    inline int width() const { return im.cols; }

    void passThrough(int x, int y);
    void display(const std::string& name, const cv::Mat& im);
    void write(int x, int y, int r = 0, int g = 0, int b = 0, bool show = true);

    void save();
    void saveSolution(std::string suffix);

    Point getStart() const;
    Point getGoal() const;

    Point findStart();
    Point findGoal();
    Point findCornerStart();
    Point findCornerGoal();

    void setStart(const Point& start);
    void setGoal(const Point& goal);
    void setElasticBandPath(const std::vector<Point>& elasticBandPath);

    const cv::Mat& getIm() const { return im; }
    const cv::Mat& getOut() const { return out; }

    void setIm(const cv::Mat& image) { im = image; }
    void setOut(const cv::Mat& output) { out = output; }

protected:
    cv::Mat im, out;
    std::string filename;
    std::vector<cv::Point> path;
    std::vector<cv::Point> path_eb;
    std::vector<std::string> windows;

    Point start_;
    Point goal_;
};

} // namespace ecn

#endif // MAZE_H
