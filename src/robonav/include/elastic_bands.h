#ifndef ELASTIC_BANDS_H
#define ELASTIC_BANDS_H

#include <maze.h>
#include <point.h>
#include <position.h>
#include <vector>

namespace ecn
{

class ElasticBand
{
public:
    ElasticBand(const std::vector<Point>& path, const Maze& maze);

    ElasticBand(const std::vector<Position>& initialPath, const Maze& maze) : maze(maze)
    {
        // Convert the Position path to a Point path
        path.resize(initialPath.size());
        for (size_t i = 0; i < initialPath.size(); ++i) {
            path[i] = Point(initialPath[i].x, initialPath[i].y);
        }
        initial_path = path;
    }

    void savePathToFile(const std::string& filename) const;

    bool optimize(const Point& start, const Point& goal); // Optimize the path using elastic band algorithm

    void showPath(int pause_inbetween) const;

    const std::vector<Point>& getPath() const;

    void repelFromObstacle(size_t idx, float rep_strength);

    std::vector<Point> gaussianSmoothing(const std::vector<Point>& path, int windowSize, float sigma);
    void generateSmoothedPath(float maxGap, int windowSize, float sigma);

    const std::vector<Point>& getSmoothedPath() { return smoothed_path; }
    const std::vector<Point>& getInitialPath() { return initial_path; }

    void updatePath(const std::vector<Position>& newPath)
    {
        path.resize(newPath.size());
        for (size_t i = 0; i < newPath.size(); ++i) {
            path[i] = Point(newPath[i].x, newPath[i].y);
        }
        initial_path = path;
        // showPath(200);
    }

    std::vector<Point> getPath() { return path; }

    void resetOptimization()
    {
        current_iteration     = 0;
        optimization_complete = false;
        total_change          = 0;
        max_interations_      = 6;
    }

    void setMaxInterations(int max_interations) { max_interations_ = max_interations; }

    int getMaxInterations() { return max_interations_; }


    bool isOptimizationComplete() const { return optimization_complete; }

    bool errorCheck() const
    {
        if (optimization_complete == true && current_iteration < max_interations_) {
            std ::cout << "Optimization error: Optimization complete but current iteration < max iterations" << std::endl;
            return true;
        } else {
            return false;
        }
    }

    void runFullOptimization(const Point& start, const Point& goal)
    {
        resetOptimization(); // Setzt den Zustand zurück

        while (!optimization_complete) {
            optimize(start, goal); // Führt eine Iteration durch
        }

        resetOptimization();

        while (!optimization_complete) {
            optimize(start, goal); // Führt eine Iteration durch
        }

        generateSmoothedPath(2.5f, 21, 1.2f);

        std::cout << "Elastic Band initial optimization completed after " << current_iteration << " iterations." << std::endl;


        resetOptimization(); // Setzt den Zustand zurück
    }

    float distanceToClosestObstacle(const Point& point, int search_radius) const;


private:
    std::vector<Point> initial_path;
    std::vector<Point> path;
    std::vector<Point> smoothed_path;
    const Maze& maze;

    int current_iteration      = 0;
    bool optimization_complete = false;
    float total_change         = 0;

    int max_interations_ = 0;

    bool resizePath(float minDistance, float maxDistance);
    // float distanceToClosestObstacle(const Point& point, int search_radius) const;

    Point computeSpringForce(size_t idx, const float spr_weight, const int radius) const;
    Point computeRepulsiveForce(size_t idx, const float rep_strength) const;
    int checkAndAjustInCorridor(size_t idx, int radius) const;
};

} // namespace ecn

#endif
