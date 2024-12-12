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
        for (size_t i = 0; i < initialPath.size(); ++i)
        {
            path[i] = Point(initialPath[i].x, initialPath[i].y);
        }
    }

    void savePathToFile(const std::string& filename) const;

    int optimize(const Point& start, const Point& goal); // Optimize the path using elastic band algorithm

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween) const;

    const std::vector<Point>& getPath() const;

    void repelFromObstacle(size_t idx, float rep_strength);

    std::vector<Point> gaussianSmoothing(const std::vector<Point>& path, int windowSize, float sigma);
    void generateSmoothedPath(const std::vector<Point>& path, float maxGap, int windowSize, float sigma);

    const std::vector<Point>& getSmoothedPath() { return smoothed_path; }

    void updatePath(const std::vector<Position>& newPath)
    {
        path.resize(newPath.size());
        for (size_t i = 0; i < newPath.size(); ++i) {
            path[i] = Point(newPath[i].x, newPath[i].y);
        }
    }


private:
    std::vector<Point> path;
    std::vector<Point> smoothed_path;
    const Maze& maze;

    bool resizePath(float minDistance, float maxDistance);
    float distanceToClosestObstacle(const Point& point, int search_radius) const;

    Point computeSpringForce(size_t idx, const float spr_weight, const int radius) const;
    Point computeRepulsiveForce(size_t idx, const float rep_strength) const;
    int checkAndAjustInCorridor(size_t idx, int radius) const;
};

} // namespace ecn

#endif
