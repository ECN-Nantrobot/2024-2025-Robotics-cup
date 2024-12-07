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

    void optimize(); // Optimize the path using elastic band algorithm

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween, const std::vector<float>& radii) const;

    float distanceToClosestObstacle(const Point& point, int search_radius) const;

    void adjustPath(float minDistance, float maxDistance);

    const std::vector<Point>& getPath() const;

private:
    std::vector<Point> path;
    const Maze& maze;  

    Point computeSpringForce(size_t idx, const float spr_weight, const int radius) const;
    Point computeRepulsiveForce(size_t idx, const float rep_radius, const float rep_strength) const;
};

} // namespace ecn

#endif
