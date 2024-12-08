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

    bool optimize(); // Optimize the path using elastic band algorithm

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween) const;

    const std::vector<Point>& getPath() const;

    void repelFromObstacle(size_t idx, float rep_strength);

private:
    std::vector<Point> path;
    const Maze& maze;

    bool adjustPath(float minDistance, float maxDistance);
    float distanceToClosestObstacle(const Point& point, int search_radius) const;

    Point computeSpringForce(size_t idx, const float spr_weight, const int radius) const;
    Point computeRepulsiveForce(size_t idx, const float rep_strength) const;
    int checkAndAjustInCorridor(size_t idx, int radius) const;
};

} // namespace ecn

#endif
