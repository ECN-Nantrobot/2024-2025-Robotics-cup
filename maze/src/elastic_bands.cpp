#include "elastic_bands.h"
#include <cmath>
#include <algorithm>
#include <fposition.h>

namespace ecn {

// Constructor
ElasticBand::ElasticBand(const std::vector<FPosition>& initialPath, const Maze& maze): path(initialPath), maze(maze) {}


void ElasticBand::fillGaps(int max_gap) {
    std::vector<FPosition> filledPath;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        FPosition current = path[i];
        FPosition next = path[i + 1];

        filledPath.push_back(current); // Keep the current point

        // Compute the distance to the next point
        float distance = std::hypot(next.x - current.x, next.y - current.y);

        // Add intermediate points if the gap is too large
        if (distance > max_gap) {
            int numIntermediatePoints = static_cast<int>(std::ceil(distance / max_gap)) - 1;
            for (int j = 1; j <= numIntermediatePoints; ++j) {
                FPosition intermediate = {
                    current.x + (float)j * (next.x - current.x) / (numIntermediatePoints + 1),
                    current.y + (float)j * (next.y - current.y) / (numIntermediatePoints + 1)
                };

                // Only add the point if it's free
                if (isPointFree(intermediate, 1)) {
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

// void ElasticBand::showPath(int iteration, int pause_inbetween, const std::vector<FPosition>& sparsePath, const int radius) const {
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
    // int i = 0;
    for (const auto& point : path) {
        // cv::Scalar color = (i == 110) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::rectangle(visualization, 
                      cv::Point(point.x * scale, point.y * scale), 
                      cv::Point((point.x + 1) * scale - grid_size, (point.y + 1) * scale - grid_size), 
                    //   color, // Red for the 50th point, Green for others
                      cv::Scalar(0, 255, 0),
                      cv::FILLED);
        // i++;

    }

    // int j = 0;
    // Draw Circles aroung the points
    for (const auto& point : path) {
        // cv::Scalar color = (j == 110) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        cv::circle(visualization, 
                   cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2), 
                   radius * scale, // Adjust radius based on scale
                //    color,
                   cv::Scalar(0, 0, 255),
                   scale / 5); // Thickness proportional to scale
        // j++;
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




std::vector<FPosition> ElasticBand::downsamplePath(int sparsity) const {
    std::vector<FPosition> sparsePath;
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

    const float alpha = 0.05;     // Step size (scaling of the total force)
    const int max_iterations = 15;  // Maximum iterations
    const float threshold = 2.1;     // Convergence threshold (number of cells that move)

    const float spring_weight = 15.0;  
    const int spring_radius = 6;         // Radius of the spring force (average FPosition of neighbors in radius)
    const float repulsive_radius = 8.0; //circle around the point
    const float repulsive_strength = 13.5;

    std::cout << "Position of point 110: (" << path[110].x << ", " << path[110].y << ")" << std::endl;

    showPath(1500, repulsive_radius);

    for (int iter = 0; iter < max_iterations; ++iter) {
        float max_change = 0;

        std::cout << "Iteration: " << iter;

        // Iterate over all points except start and end
        for (size_t i = 1; i < path.size() - 1; ++i) {

            FPosition spring_force = computeSpringForce(i, spring_weight, spring_radius);
            FPosition repulsive_force = computeRepulsiveForce(i, repulsive_radius, repulsive_strength);

            FPosition displacement = {
                alpha * (spring_force.x + repulsive_force.x),
                alpha * (spring_force.y + repulsive_force.y)
            };

            // if(i == 110){
            //     std::cout << "spring_force: " << spring_force << ", repulsive_force: " << repulsive_force << ", displacement: "  << displacement << ", pos x: " << path[i].x <<  ", pos y: " << path[i].y ;
            // }

            FPosition newPos = {
                path[i].x + displacement.x,
                path[i].y + displacement.y
            };

            // if(i == 110){
            //     if (!isPointFree(newPos, 0)){
            //         std::cout << "    nottttt   freeeeeee  newPos: " << newPos << std::endl;
            //     }
            // }
            
            // Update the path point with safety checks
            if (isPointFree(newPos, 0)) {  // Check if the new position is not inside an obstacle
                // Check if the new position is within the maze bounds

                // if(newPos.x <= 0){
                //     newPos.x = 0;
                // }
                // else if(newPos.x >= maze.width()){
                //     newPos.x = maze.width();
                // }
                // else if(newPos.y <= 0){
                //     newPos.y = 0;
                // }
                // else if(newPos.y >= maze.height()){
                //     newPos.y = maze.height();
                // }
                // else{

                // if (isWithinBounds(newPos)) {
                    max_change = std::max(max_change,
                        std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
                    path[i] = newPos;  // Update the path point to the new position
                // }

                }   
            



            // // if (isPointFree(newPos)) {
            //     max_change = std::max(max_change, std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
            //     path[i] = newPos;
            // // }
        }

        showPath(750, repulsive_radius);

        std::cout << ", max_change: " << max_change << std::endl;

        if (max_change <= threshold) {
            std::cout << "Optimization converged after " << max_iterations << " iterations, becuase: max_change vs threshold: " << max_change << " vs " << threshold << std::endl;
            break;
        }
    }

    std::cout << "Optimization converged after " << max_iterations <<  std::endl;
    
    fillGaps(1); // Fill gaps after optimization (maybe not needed later in real world)
}


FPosition ElasticBand::computeSpringForce(size_t idx, const float spr_weight, int radius) const {
    FPosition current = path[idx];
    FPosition force = {0, 0};
    int count = 0; //used to calculate the number of neighbors contributing to the force: so that spring force pulls the current point toward the average FPosition of its neighbors

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
                FPosition neighbor = path[neighbor_idx];
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

FPosition ElasticBand::computeRepulsiveForce(size_t idx, const float rep_radius, const float rep_strength) const {
    FPosition current = path[idx];
    FPosition force = {0, 0};

    // Check surrounding cells in the radius (actually square but we remove the corners)
    for (int dx = -rep_radius; dx <= rep_radius; ++dx) {
        for (int dy = -rep_radius; dy <= rep_radius; ++dy) {
            float distance = std::hypot(dx, dy);

            // Include only cells within the circular radius
            if (distance > 0 && distance <= rep_radius) {
                FPosition neighbor = {current.x + dx, current.y + dy};

                if(idx == 110){
                    //std::cout << "dx: " << dx << " ,dy: " << dy << std::endl;
                }

                // If the neighbor is an obstacle
                if (!isPointFree(neighbor, 0)) {
                    if(idx == 110){
                        //std::cout << "neighbor: " << neighbor << ", current: " << current << ", distance: " << distance << ", rep_radius: " << rep_radius << ", rep_strength: " << rep_strength << std::endl;
                    }
                    float scale = rep_strength / (distance * distance);

                    // Accumulate force
                    force.x -= dx * scale;
                    force.y -= dy * scale;

                    if(idx == 110){
                       // std::cout << "force: " << force << std::endl;
                    }
                }
            }
        }
    }

    return force;
}

bool ElasticBand::isPointFree(const FPosition& pos, float bufferRadius) const {
    // Cast FPosition to integers for Maze's isPointFree
    int x = static_cast<int>(std::round(pos.x));
    int y = static_cast<int>(std::round(pos.y));

    // Check if the position is free
    if (!maze.isFree(x, y)) {
        return false;  // Position is blocked by an obstacle
    }

    // Check if the position is within a buffer radius from any obstacles
    for (int dx = -static_cast<int>(bufferRadius); dx <= static_cast<int>(bufferRadius); ++dx) {
        for (int dy = -static_cast<int>(bufferRadius); dy <= static_cast<int>(bufferRadius); ++dy) {
            if (!maze.isFree(x + dx, y + dy)) {
                return false;  // The point is too close to an obstacle
            }
        }
    }

    return true;
}

// bool ElasticBand::isWithinBounds(const FPosition& pos) const {
//     // Cast FPosition to integers for bounds checking
//     int x = static_cast<int>(std::round(pos.x));
//     int y = static_cast<int>(std::round(pos.y));

//     // Check if the position is within the maze dimensions
//     return (x >= 0 && x < maze.width() && y >= 0 && y < maze.height());
// }





// bool ElasticBand::isFree(const FPosition& pos) const {
//      return maze.isFree(static_cast<int>(pos.x), static_cast<int>(pos.y))    
//     ;
// }

const std::vector<FPosition>& ElasticBand::getPath() const {
    return path;
}

}
