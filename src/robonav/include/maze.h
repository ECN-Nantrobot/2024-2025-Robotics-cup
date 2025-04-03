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
    // void loadLowRes(std::string _filename);


    bool isFree(float fx, float fy, bool lowres = false) const;
    bool isFree(const Point& p, bool lowres = false) const;
    bool isFreeNotPermanent(float fx, float fy, bool lowres=false) const;
    bool isFreeNotPermanent(const Point& p, bool lowres = false) const;


    inline int height() const
    {
        return im.rows;
    }
    inline int width() const { return im.cols; }

    void passThrough(int x, int y);
    void display(const std::string& name, const std::string& type);
    void write(int x, int y, int r = 0, int g = 0, int b = 0, bool show = true);
    void clear();

    void save();
    void saveSolution(std::string suffix, const std::vector<Point>& astar_path, const std::vector<Point>& eb_path);

    Point getStart() const;
    Point getGoal() const;

    Point findStart();
    Point findGoal();

    void resetIm() { im = im_original.clone(); }
    void setPixel(int x, int y, const cv::Vec3b& color) { im.at<cv::Vec3b>(y, x) = color; }

    void setStart(const Point& start);
    void setGoal(const Point& goal);
    void setElasticBandPath(const std::vector<Point>& elasticBandPath);

    const cv::Mat& getIm() const { return im; }
    const cv::Mat& getOut() const { return out; }

    void setIm(const cv::Mat& image) { im = image; }
    void setOut(const cv::Mat& output) { out = output; }

    void computeDistanceTransform();
    float getDistanceToObstacle(const Point& p) const;
    void visualizeDistanceTransform() const;


    cv::Mat im;
    cv::Mat out;
    cv::Mat im_lowres;
    cv::Mat out_lowres;


    float resize_for_astar = 0.3;


protected:
    cv::Mat im_original;
    std::string filename;
    std::vector<cv::Point> path;
    std::vector<cv::Point> path_eb;
    std::vector<std::string> windows;

    cv::Mat distanceTransformMap; // Stores the distance transform
    cv::Mat resizedDistanceTransform;
    cv::Mat binaryMapLowres;
    cv::Mat binaryMap;


    Point start_;
    Point goal_;

    const int permanent_obstacle_treshold = 50;
    const int obstacle_treshold = 120;
};

} // namespace ecn

#endif // MAZE_H
