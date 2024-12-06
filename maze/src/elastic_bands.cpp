#include "elastic_bands.h"
#include <cmath>
#include <algorithm>
#include <maze.h>
#include <point.h>
#include <fstream>
#include <iostream>


namespace ecn {

// Constructor
ElasticBand::ElasticBand(const std::vector<Point>& initialPath, const Maze& maze): path(initialPath), maze(maze) {}

void ElasticBand::savePathToFile(const std::string &filename) const{
    std::ofstream outFile(filename);
    if (!outFile.is_open()){
        throw std::runtime_error("Could not open file for saving: " + filename);
    }

    for (const auto &pos : path){
        outFile << pos.x << " " << pos.y << "\n";
    }
    std::cout << "Path saved successfully to " << filename << std::endl;
}

void ElasticBand::fillGaps(int max_gap) {
    std::vector<Point> filledPath;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        Point current = path[i];
        Point next = path[i + 1];

        filledPath.push_back(current); // Keep the current point

        // Compute the distance to the next point
        float distance = std::hypot(next.x - current.x, next.y - current.y);

        // Add intermediate points if the gap is too large
        if (distance > max_gap) {
            int numIntermediatePoints = static_cast<int>(std::ceil(distance / max_gap)) - 1;
            for (int j = 1; j <= numIntermediatePoints; ++j) {
                Point intermediate = {
                    current.x + (float)j * (next.x - current.x) / (numIntermediatePoints + 1),
                    current.y + (float)j * (next.y - current.y) / (numIntermediatePoints + 1)
                };

                // Only add the point if it's free
                if (isPointFree(intermediate)) {
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
                      cv::Scalar(0, 255, 0),
                      cv::FILLED);
    }

    // Draw Circles aroung the points
    for (const auto& point : path) {
        cv::circle(visualization, 
                   cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2), 
                   radius * scale, // Adjust radius based on scale
                   cv::Scalar(0, 0, 255),
                   scale / 5); // Thickness proportional to scale
    }

    // Display the visualization
    std::string window_name = "Elastic Band Optimization, resolution " + std::to_string(scale) + ":1";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1500, 1200);
    cv::moveWindow(window_name, 50, 50);
    cv::imshow(window_name, visualization); 
    cv::waitKey(pause_inbetween);
}

std::vector<Point> ElasticBand::downsamplePath(int sparsity) const {
    std::vector<Point> sparsePath;
    for (size_t i = 0; i < path.size(); i += sparsity) {
        sparsePath.push_back(path[i]);
    }
    // Ensure the last point (goal) is included
    if (path.back() != sparsePath.back()) {
        sparsePath.push_back(path.back());
    }
    return sparsePath;
}


void ElasticBand::optimize(int sparsity) {

    path = downsamplePath(sparsity);

    const float alpha = 0.05;     // Step size (scaling of the total force)
    const int max_iterations = 15;  // Maximum iterations
    const float threshold = 2.1;     // Convergence threshold (number of cells that move)
    float max_change = 0;

    const float spring_weight = 15.0;  
    const int spring_radius = 6;         // Radius of the spring force (average Point of neighbors in radius)
    const float repulsive_radius = 8.0; //circle around the point
    const float repulsive_strength = 13.5;

    showPath(1500, repulsive_radius);

    for (int iter = 0; iter < max_iterations; ++iter) {
        max_change = 0;

        std::cout << "Iteration: " << iter;

        // Iterate over all points except start and end
        for (size_t i = 1; i < path.size() - 1; ++i) {

            Point spring_force = computeSpringForce(i, spring_weight, spring_radius);
            Point repulsive_force = computeRepulsiveForce(i, repulsive_radius, repulsive_strength);

            Point displacement = {
                alpha * (spring_force.x + repulsive_force.x),
                alpha * (spring_force.y + repulsive_force.y)
            };

            Point newPos = {
                path[i].x + displacement.x,
                path[i].y + displacement.y
            };
            
            if (isPointFree(newPos)) {

                //maybe add check if the new position is outside of the maze bounds or inside an obstacle, otherwise the points are stuck if the displacement is to high

                max_change = std::max(max_change,
                        std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
                    path[i] = newPos;  // Update the path point to the new position
           }   
        }

        showPath(750, repulsive_radius);

        std::cout << ", max_change: " << max_change << std::endl;

        if (max_change <= threshold) {
            std::cout << "Optimization converged after " << max_iterations << " iterations, becuase: max_change vs threshold: " << max_change << " vs " << threshold << std::endl;
            break;
        }
    }
    
    if (max_change > threshold){std::cout << "Optimization eded because of max iterations: " << max_iterations <<  std::endl;}
    
    fillGaps(1); // Fill gaps after optimization (maybe not needed later in real world)
}


Point ElasticBand::computeSpringForce(size_t idx, const float spr_weight, int radius) const {
    Point current = path[idx];
    Point force = {0, 0};
    int count = 0; //used to calculate the number of neighbors contributing to the force: so that spring force pulls the current point toward the average Point of its neighbors

    // Special handling for the first and last movable points (send and second last point)
    if (idx == 1) {
            force.x += (path[0].x - path[idx].x) * spr_weight; // Pull towards start
            force.y += (path[0].y - path[idx].y) * spr_weight;
    } else if (idx == path.size() - 2) {
        force.x += (path.back().x - path[idx].x) * spr_weight; // Pull towards end, 0.5 added to pampen because of high forces
        force.y += (path.back().y - path[idx].y) * spr_weight;
    } else {

        // Iterate over the points within the radius
        for (int i = -radius; i <= radius; ++i) {
            if (i == 0) continue; // Skip the current point

            int neighbor_idx = idx + i;
            if (neighbor_idx >= 0 && neighbor_idx < path.size()) {
                Point neighbor = path[neighbor_idx];
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

Point ElasticBand::computeRepulsiveForce(size_t idx, const float rep_radius, const float rep_strength) const {
    Point current = path[idx];
    Point force = {0, 0};

    // Check surrounding cells in the radius (actually square but we remove the corners)
    for (int dx = -rep_radius; dx <= rep_radius; ++dx) {
        for (int dy = -rep_radius; dy <= rep_radius; ++dy) {
            float distance = std::hypot(dx, dy);

            // Include only cells within the circular radius
            if (distance > 0 && distance <= rep_radius) {
                Point neighbor = {current.x + dx, current.y + dy};

                if (!isPointFree(neighbor)) {
                    float scale = rep_strength / (distance * distance);

                    force.x -= dx * scale;
                    force.y -= dy * scale;
                }
            }
        }
    }

    return force;
}

bool ElasticBand::isPointFree(const Point& pos) const {
    int x = static_cast<int>(std::round(pos.x));
    int y = static_cast<int>(std::round(pos.y));

    if (!maze.isFree(x, y)) {
        return false;
    }
    return true;
}

const std::vector<Point>& ElasticBand::getPath() const {
    return path;
}

}
