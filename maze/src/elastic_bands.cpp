#include "elastic_bands.h"
#include <cmath>
#include <algorithm>

namespace ecn {

// Constructor
ElasticBand::ElasticBand(const std::vector<Position>& initialPath, const Maze& maze): path(initialPath), maze(maze) {}


void ElasticBand::fillGaps(int max_gap) {
    std::vector<Position> filledPath;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        Position current = path[i];
        Position next = path[i + 1];

        filledPath.push_back(current); // Keep the current point

        // Compute the distance to the next point
        double distance = std::hypot(next.x - current.x, next.y - current.y);

        // Add intermediate points if the gap is too large
        if (distance > max_gap) {
            int numIntermediatePoints = static_cast<int>(std::ceil(distance / max_gap)) - 1;
            for (int j = 1; j <= numIntermediatePoints; ++j) {
                Position intermediate = {
                    current.x + j * (next.x - current.x) / (numIntermediatePoints + 1),
                    current.y + j * (next.y - current.y) / (numIntermediatePoints + 1)
                };

                // Only add the point if it's free
                if (isFree(intermediate)) {
                    filledPath.push_back(intermediate);
                }
            }
        }
    }

    // Add the last point
    filledPath.push_back(path.back());

    // Replace the original path with the filled path
    path = filledPath;
}

// void ElasticBand::showPath(int iteration, int pause_inbetween, const std::vector<Position>& sparsePath, const int radius) const {
//     // Create a copy of the maze to draw the path
//     cv::Mat visualization = maze.getOut().clone();

//     cv::Vec3b pathColor(0, 255, 0);     // Green for the dense path
//     cv::Scalar ringColor(0, 0, 255);    // Red for the ring

//     // Draw the dense path
//     for (const auto& point : path) {
//         visualization.at<cv::Vec3b>(point.y, point.x) = pathColor;
//     }

//     // Highlight the sparse points with rings
//     for (const auto& point : sparsePath) {
//         // Draw a ring (not a filled circle)
//         int thickness = 0; // Positive thickness for the outline
//         cv::circle(visualization, cv::Point(point.x, point.y), radius, ringColor, thickness);
//     }

//     std::string window_name = "Elastic Band Optimization";
//     cv::namedWindow(window_name, cv::WINDOW_NORMAL); // Allow resizing
//     cv::resizeWindow(window_name, 1000, 800);       // Resize to 1000x800 pixels

//     cv::imshow(window_name, visualization);

//     // Wait briefly or until a key is pressed
//     cv::waitKey(pause_inbetween);
// }

void ElasticBand::showPath(int pause_inbetween, const int radius) const {
    const int scale = 10; // Scale factor: Each grid cell becomes 10x10 pixels
    const int grid_size = 1; // Grid line thickness


    // Create a higher-resolution blank image
    cv::Mat visualization(maze.getOut().rows * scale, maze.getOut().cols * scale, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int y = 0; y < maze.getOut().rows; ++y) {
        for (int x = 0; x < maze.getOut().cols; ++x) {
            if (!maze.isFree(x, y)) {
                cv::rectangle(visualization, 
                              cv::Point(x * scale, y * scale), 
                              cv::Point((x + 1) * scale, (y + 1) * scale), 
                              cv::Scalar(0, 0, 0), // Black for obstacles
                              cv::FILLED);
            }
        }
    }

    // Draw the points
    for (const auto& point : path) {
        cv::rectangle(visualization, 
                      cv::Point(point.x * scale, point.y * scale), 
                      cv::Point((point.x + 1) * scale - grid_size, (point.y + 1) * scale - grid_size), 
                      cv::Scalar(0, 255, 0), // Green for the path
                      cv::FILLED);
    }

    // Draw Circles aroung the points
    for (const auto& point : path) {
        cv::circle(visualization, 
                   cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2), 
                   radius * scale, // Adjust radius based on scale
                   cv::Scalar(0, 0, 255), // Red for the ring
                   scale / 5); // Thickness proportional to scale
    }

    // Display the visualization
    std::string window_name = "Elastic Band Optimization, resolution " + std::to_string(scale) + ":1";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1500, 1200);
    cv::moveWindow(window_name, 50, 50);
    cv::imshow(window_name, visualization); 


    // Wait briefly or until a key is pressed
    cv::waitKey(pause_inbetween);
}




std::vector<Position> ElasticBand::downsamplePath(int sparsity) const {
    std::vector<Position> sparsePath;
    for (size_t i = 0; i < path.size(); i += sparsity) {
        sparsePath.push_back(path[i]);
    }
    // Ensure the last point (goal) is included
    if (path.back() != sparsePath.back()) {
        sparsePath.push_back(path.back());
    }
    return sparsePath;
}


// Optimize the path using elastic band algorithm
void ElasticBand::optimize(int sparsity) {

    path = downsamplePath(sparsity);

    const double alpha = 0.072;     // Step size (scaling of the total force)
    const int max_iterations = 30;  // Maximum iterations
    const double threshold = 1.5;     // Convergence threshold (number of cells that move)

    const double spring_weight = 10.0;  
    const int spring_radius = 5;         // Radius of the spring force (average position of neighbors in radius)
    const double repulsive_radius = 8.0; //circle around the point
    const double repulsive_strength = 13.0;
    // double dynamicSpringWeight = 10.0; // Start with a small spring weight

    showPath(2000, repulsive_radius);

    for (int iter = 0; iter < max_iterations; ++iter) {
        double max_change = 0;

        std::cout << "Iteration: " << iter;
        // dynamicSpringWeight = std::min(dynamicSpringWeight + 0.5, 20.0); // Gradually increase

        // Iterate over all points except start and end
        for (size_t i = 1; i < path.size() - 1; ++i) {

            Position spring_force = computeSpringForce(i, spring_weight, spring_radius);
            Position repulsive_force = computeRepulsiveForce(i, repulsive_radius, repulsive_strength);

            // Combine forces
            Position displacement = {
                static_cast<int> (alpha * (spring_force.x + repulsive_force.x)),
                static_cast<int> (alpha * (spring_force.y + repulsive_force.y))
            };

            // Update the position if it's free
            Position newPos = {
                path[i].x + displacement.x,
                path[i].y + displacement.y
            };

            if (isFree(newPos)) {
                max_change = std::max(max_change,
                    std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
                path[i] = newPos;
            }

            
        }

        showPath(400, repulsive_radius);

        // std::cout << ", spring_force: " << dynamicSpringWeight;

        std::cout << ", max_change: " << max_change << std::endl;

        // Stop if the path converges
        if (max_change <= threshold) {
            std::cout << "Optimization converged after " << max_iterations << " iterations, becuase: max_change vs threshold: " << max_change << " vs " << threshold << std::endl;
            break;
        }
    }

    std::cout << "Optimization converged after " << max_iterations <<  std::endl;
    
    // Fill gaps after optimization (maybe not needed later in real world)
    fillGaps(1);
}

Position ElasticBand::computeSpringForce(size_t idx, const double spr_weight, int radius) const {
    Position current = path[idx];
    Position force = {0, 0};
    int count = 0; //used to calculate the number of neighbors contributing to the force: so that spring force pulls the current point toward the average position of its neighbors
    double damping = 0.5; 

    // Special handling for the first and last movable points (send and second last point)
    if (idx == 1) {
            force.x += (path[0].x - path[idx].x) * spr_weight * damping; // Pull towards start
            force.y += (path[0].y - path[idx].y) * spr_weight * damping;
    } else if (idx == path.size() - 2) {
        force.x += (path.back().x - path[idx].x) * spr_weight * damping; // Pull towards end, 0.5 added to pampen because of high forces
        force.y += (path.back().y - path[idx].y) * spr_weight * damping;
    } else {

        // Iterate over the points within the radius
        for (int i = -radius; i <= radius; ++i) {
            if (i == 0) continue; // Skip the current point

            int neighbor_idx = idx + i;
            if (neighbor_idx >= 0 && neighbor_idx < path.size()) {
                Position neighbor = path[neighbor_idx];
                force.x += neighbor.x;
                force.y += neighbor.y;
                count++;
            }
        }

        if (count > 0) {
            force.x = (force.x / count - current.x) * spr_weight;
            force.y = (force.y / count - current.y) * spr_weight;
        }
    }
    return force;
}

Position ElasticBand::computeRepulsiveForce(size_t idx, const double rep_radius, const double rep_strength) const {
    Position current = path[idx];
    Position force = {0, 0};

    // Check surrounding cells in the radius (actually square but we remove the corners)
    for (int dx = -rep_radius; dx <= rep_radius; ++dx) {
        for (int dy = -rep_radius; dy <= rep_radius; ++dy) {
            double distance = std::hypot(dx, dy);

            // Include only cells within the circular radius
            if (distance > 0 && distance <= rep_radius) {
                Position neighbor = {current.x + dx, current.y + dy};

                // If the neighbor is an obstacle
                if (!isFree(neighbor)) {
                    double scale = rep_strength / (distance * distance);

                    // Accumulate force
                    force.x -= dx * scale;
                    force.y -= dy * scale;
                }
            }
        }
    }

    return force;
}

bool ElasticBand::isFree(const Position& pos) const {
    return maze.isFree(static_cast<int>(pos.x), static_cast<int>(pos.y));
}

const std::vector<Position>& ElasticBand::getPath() const {
    return path;
}

}
