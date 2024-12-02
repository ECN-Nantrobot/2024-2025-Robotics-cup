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
    void optimize(int sparsity);

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween, const int radius) const;


    // Retrieve the optimized path
    const std::vector<Position>& getPath() const;

private:
    std::vector<Position> path; // Path to be optimized
    const Maze& maze;           // Reference to the maze

    std::vector<Position> downsamplePath(int sparsity = 1) const;

    // Compute the internal spring force (gluing force)
    Position computeSpringForce(size_t idx, const double spr_weight, const int radius) const;

    // Compute the repulsive force to avoid obstacles
    Position computeRepulsiveForce(size_t idx, const double rep_radius, const double rep_strength) const;

    // Check if a point is within free space
    bool isFree(const Position& pos) const;
};

}

#endif
