#ifndef ELASTIC_BANDS_H
#define ELASTIC_BANDS_H

#include <vector>
#include <position.h>
#include <maze.h>

namespace ecn {

class ElasticBand {
public:
    ElasticBand(const std::vector<Position>& path, const Maze& maze);

    // Optimize the path using elastic band algorithm
    void optimize();

    // Retrieve the optimized path
    const std::vector<Position>& getPath() const;

private:
    std::vector<Position> path; // Path to be optimized
    const Maze& maze;           // Reference to the maze

    // Compute the internal spring force (gluing force)
    Position computeSpringForce(size_t idx) const;

    // Compute the repulsive force to avoid obstacles
    Position computeRepulsiveForce(size_t idx) const;

    // Check if a point is within free space
    bool isFree(const Position& pos) const;
};

}

#endif
