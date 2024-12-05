#ifndef ELASTIC_BANDS_H
#define ELASTIC_BANDS_H

#include <vector>
#include <position.h>
#include <maze.h>

#include <fposition.h>

namespace ecn {

class ElasticBand {
public:
    ElasticBand(const std::vector<FPosition>& path, const Maze& maze);

    // Optimize the path using elastic band algorithm
    void optimize(int sparsity);

    void fillGaps(int maxGap);

    void showPath(int pause_inbetween, const int radius) const;


    // Retrieve the optimized path
    const std::vector<FPosition>& getPath() const;

private:
    std::vector<FPosition> path; // Path to be optimized
    const Maze& maze;           // Reference to the maze

    std::vector<FPosition> downsamplePath(int sparsity = 1) const;

    // Compute the internal spring force (gluing force)
    FPosition computeSpringForce(size_t idx, const float spr_weight, const int radius) const;

    // Compute the repulsive force to avoid obstacles
    FPosition computeRepulsiveForce(size_t idx, const float rep_radius, const float rep_strength) const;

    // Check if a point is within free space
    bool isPointFree(const FPosition& pos) const;

    bool isWithinBounds(const FPosition& pos) const;
};

}

#endif
