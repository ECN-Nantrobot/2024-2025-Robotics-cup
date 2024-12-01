#include "elastic_bands.h"
#include <cmath>
#include <algorithm>

namespace ecn {

// Constructor
ElasticBand::ElasticBand(const std::vector<Position>& initialPath, const Maze& maze)
    : path(initialPath), maze(maze) {}

// Optimize the path using elastic band algorithm
void ElasticBand::optimize() {
    const double alpha = 0.1;        // Step size
    const int maxIterations = 100;  // Maximum iterations
    const double threshold = 1e-3;  // Convergence threshold

    for (int iter = 0; iter < maxIterations; ++iter) {
        double maxChange = 0;

        // Iterate over all points except start and end
        for (size_t i = 1; i < path.size() - 1; ++i) {
            Position springForce = computeSpringForce(i);
            Position repulsiveForce = computeRepulsiveForce(i);

            // Combine forces
            Position displacement = {
                alpha * (springForce.x + repulsiveForce.x),
                alpha * (springForce.y + repulsiveForce.y)
            };

            // Update the position if it's free
            Position newPos = {
                path[i].x + displacement.x,
                path[i].y + displacement.y
            };

            if (isFree(newPos)) {
                maxChange = std::max(maxChange,
                    std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
                path[i] = newPos;
            }
        }

        // Stop if the path converges
        if (maxChange < threshold) {
            break;
        }
    }
}

// Compute the spring force
Position ElasticBand::computeSpringForce(size_t idx) const {
    Position prev = path[idx - 1];
    Position next = path[idx + 1];
    Position current = path[idx];

    return {
        (prev.x + next.x - 2 * current.x),  // Pull towards neighbors
        (prev.y + next.y - 2 * current.y)
    };
}

// Compute the repulsive force to avoid obstacles
Position ElasticBand::computeRepulsiveForce(size_t idx) const {
    const double repulsionRadius = 3.0; // Radius for obstacle avoidance
    const double repulsionStrength = 10.0;

    Position current = path[idx];
    Position force = {0, 0};

    // Check surrounding cells
    for (int dx = -repulsionRadius; dx <= repulsionRadius; ++dx) {
        for (int dy = -repulsionRadius; dy <= repulsionRadius; ++dy) {
            Position neighbor = {current.x + dx, current.y + dy};

            if (!isFree(neighbor)) {
                double distance = std::hypot(dx, dy);
                if (distance > 0 && distance <= repulsionRadius) {
                    double scale = repulsionStrength / (distance * distance);
                    force.x -= dx * scale;
                    force.y -= dy * scale;
                }
            }
        }
    }

    return force;
}

// Check if a position is free
bool ElasticBand::isFree(const Position& pos) const {
    return maze.isFree(static_cast<int>(pos.x), static_cast<int>(pos.y));
}

// Retrieve the optimized path
const std::vector<Position>& ElasticBand::getPath() const {
    return path;
}

}
