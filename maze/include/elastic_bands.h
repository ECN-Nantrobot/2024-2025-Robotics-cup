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

    // Optimize the path using elastic band algorithm
    void optimize(int sparsity);

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween, const std::vector<float>& radii) const;

    float distanceToClosestObstacle(const Point& point, int search_radius) const;

    // Retrieve the optimized path
    const std::vector<Point>& getPath() const;

private:
    std::vector<Point> path; // Path to be optimized
    const Maze& maze;        // Reference to the maze

    std::vector<Point> downsamplePath(int sparsity = 1) const;

    // Compute the internal spring force (gluing force)
    Point computeSpringForce(size_t idx, const float spr_weight, const int radius) const;

    // Compute the repulsive force to avoid obstacles
    Point computeRepulsiveForce(size_t idx, const float rep_radius, const float rep_strength) const;

    // Check if a point is within free space
    bool isPointFree(const Point& pos) const;
};

} // namespace ecn

#endif
