#include "elastic_bands.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <maze.h>
#include <numeric>
#include <point.h>
#include <thread>

namespace ecn
{

// Constructor
ElasticBand::ElasticBand(const std::vector<Point>& initialPath, const Maze& maze) : path(initialPath), maze(maze) {}

void ElasticBand::savePathToFile(const std::string& filename) const
{
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        throw std::runtime_error("Could not open file for saving: " + filename);
    }

    for (const auto& pos : smoothed_path) {
        outFile << pos.x << " " << pos.y << "\n";
    }
    std::cout << "Path saved successfully to " << filename << std::endl;
}

std::vector<Point> ElasticBand::gaussianSmoothing(const std::vector<Point>& path, int windowSize, float sigma)
{
    if (windowSize % 2 == 0) {
        std::cerr << "Window size must be odd. Adjusting to " << windowSize + 1 << "." << std::endl;
        ++windowSize;
    }

    std::vector<Point> smoothedPath;
    int halfWindow = windowSize / 2;
    std::vector<float> kernel(windowSize);

    // Generate Gaussian Kernel
    float sum = 0.0f;
    for (int i = -halfWindow; i <= halfWindow; ++i) {
        kernel[i + halfWindow] = std::exp(-(i * i) / (2 * sigma * sigma));
        sum += kernel[i + halfWindow];
    }

    // Normalize the kernel
    for (auto& value : kernel) {
        value /= sum;
    }

    for (size_t i = 0; i < path.size(); ++i) {
        if (i < halfWindow || i >= path.size() - halfWindow) {
            smoothedPath.push_back(path[i]); // Keep the original point
            continue;
        }

        float sumX = 0.0f, sumY = 0.0f;

        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = i + j;
            sumX += path[idx].x * kernel[j + halfWindow];
            sumY += path[idx].y * kernel[j + halfWindow];
        }

        smoothedPath.push_back(Point{ sumX, sumY });
    }

    return smoothedPath;
}

void ElasticBand::generateSmoothedPath(float maxGap, int windowSize, float sigma)
{
    std::vector<Point> filledPath;

    // Fill gaps first
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Point current = path[i];
        Point next    = path[i + 1];

        filledPath.push_back(current);

        float distance = std::hypot(next.x - current.x, next.y - current.y);

        if (distance > maxGap) {
            int numOfIntermPoints = static_cast<int>(std::ceil(distance / maxGap)) - 1;
            for (int j = 1; j <= numOfIntermPoints; ++j) {
                Point intermediate = { current.x + (float)j * (next.x - current.x) / (numOfIntermPoints + 1), current.y + (float)j * (next.y - current.y) / (numOfIntermPoints + 1) };
                filledPath.push_back(intermediate);
            }
        }
    }

    // Add the last point
    filledPath.push_back(path.back());

    smoothed_path = gaussianSmoothing(filledPath, windowSize, sigma);
}

void ElasticBand::showPath(int pause_inbetween) const
{
    const int scale     = 15; // Scale factor: Each grid cell becomes ...x... pixels

    // Create a higher-resolution blank image
    cv::Mat visualization;
    cv::resize(maze.getIm(), visualization, cv::Size(), scale, scale, cv::INTER_NEAREST);

    // Draw the points
    for (const auto& point : path) {
        cv::rectangle(visualization, cv::Point(point.x * scale, point.y * scale), cv::Point((point.x + 1) * scale, (point.y + 1) * scale), point.colour, cv::FILLED);
    }

    // Draw the first point bigger
    const auto& first_point = path.front();
    cv::circle(visualization, cv::Point(first_point.x * scale + scale / 2, first_point.y * scale + scale / 2),
               3 * scale, // Adjust radius based on scale and extra_scale
               first_point.colour,
               cv::FILLED); // Filled circle

    // Draw Circles aroung the points
    for (const auto& point : path) {
        cv::circle(visualization, cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2),
                   point.radius * scale, // Adjust radius based on scale
                   point.colour,
                   scale / 5); // Thickness proportional to scale
    }

    std::string window_name = "Elastic Band Optimization, resolution " + std::to_string(scale) + ":1";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1800, 1400);
    cv::moveWindow(window_name, 50, 50);
    cv::imshow(window_name, visualization);
    for (int i = 0; i < pause_inbetween; i++) {
        if (cv::waitKey(1) == 2) {
            break;
        }
    }
}

float ElasticBand::distanceToClosestObstacle(const Point& point, int search_radius) const
{
    // Use the Maze's precomputed distance transform for fast lookup
    return maze.getDistanceToObstacle(point);
}

// float ElasticBand::distanceToClosestObstacle(const Point& point, int search_radius) const
// {
//     float min_distance = search_radius;

//     // Iterate over the local area around the point
//     for (int dx = -search_radius; dx <= search_radius; ++dx) {
//         for (int dy = -search_radius; dy <= search_radius; ++dy) {
//             int nx = point.x + dx;
//             int ny = point.y + dy;

//             if (nx < 0 || nx >= maze.width() || ny < 0 || ny >= maze.height()) {
//                 continue;
//             }

//             if (!maze.isFree(nx, ny)) {
//                 float distance = std::hypot(dx, dy);
//                 if (distance < min_distance) {
//                     min_distance = distance;
//                 }
//             }
//         }
//     }

//     return min_distance == std::numeric_limits<float>::max() ? -1.0f : min_distance; // Return -1 if no obstacle found
// }

bool ElasticBand::resizePath(float min_dist, float max_dist)
{
    int max_path_size = std::min(1000, static_cast<int>(path.size()) * 2);
    std::vector<Point> adjustedPath(max_path_size * 1.4);
    size_t i_adjusted  = 0;
    size_t num_of_skip = 0;

    adjustedPath[i_adjusted++] = path.front();

    // std ::cout << "Path.size: " << path.size();
    size_t i = 0;
    while (i + num_of_skip < path.size() - 1) {
        Point current = path[i];
        Point next    = path[i + 1 + num_of_skip];

        float distance = std::hypot(next.x - current.x, next.y - current.y);


        if (distance >= min_dist && distance <= max_dist) {
            adjustedPath[i_adjusted++] = next; // increment after assignement
            i++;
            i           = i + num_of_skip;
            num_of_skip = 0;
        }

        else if (distance > max_dist) {
            int num_of_interm_points = std::min(static_cast<int>(std::ceil(distance / max_dist)) - 1, 5);

            for (int j = 1; j <= num_of_interm_points; ++j) {
                Point intermediate = { current.x + (float)j * (next.x - current.x) / (num_of_interm_points + 1), current.y + (float)j * (next.y - current.y) / (num_of_interm_points + 1) };
                adjustedPath[i_adjusted++] = intermediate;
            }
            adjustedPath[i_adjusted++] = next;
            i++;
            i           = i + num_of_skip;
            num_of_skip = 0;
        } else {
            num_of_skip++;
        }
    }

    adjustedPath[i_adjusted++] = path.back();
    // std ::cout << "Path.size: " << path.size();

    adjustedPath.resize(i_adjusted);
    path = adjustedPath;
    if (path.size() > max_path_size) {
        return false;
    }
    // std ::cout << " -> " << path.size();
    return true;
}

bool ElasticBand::optimize(const Point& start, const Point& goal)
{

    if (optimization_complete) {
        return true; // Optimization already complete
    }

    static int show_time = 0;

    const float alpha                  = 0.082; // Step size (scaling of the total force)
    const int max_iterations           = max_interations_;
    const float total_change_threshold = 0.0003; //(total distanc of movement of points)
    float total_change                 = 0;

    const float spring_weight_default = 16.5;
    const int spring_radius           = 1; // Radius of the spring force (average Point of neighbors in radius)
    int dynamic_spring_radius         = spring_radius;
    float rep_to_spring_radius_factor = 0.4;

    const float repulsive_strength = 6;
    const int min_rep_radius       = 6;
    const int max_rep_radius       = 18;
    float dynamic_rep_radius       = min_rep_radius;
    const int repel_raduis         = 24;
    const float repel_variation    = 0.3; // +- 40%

    const int small_gap_radius = 6;

    float min_distance = 3.2;
    float max_distance = 4.2;

    path.front() = start;
    path.back()  = goal;

    std::vector<long long> dynamicSpringWeightTimes;
    std::vector<long long> computeRepulsiveForceTimes;

    int total_radius_time    = 0;
    int total_repulsive_time = 0;

    total_change = 0;

    if (current_iteration % 10 == 0) {
        if (!resizePath(min_distance, max_distance)) { // 20 - 30 microseconds
            std::cerr << "Optimization exited early: resizePath too long" << std::endl;
            return false;
        }
    }

    for (size_t i = 1; i < path.size() - 2; ++i) {

        // auto start_time = std::chrono::high_resolution_clock::now();

        float dynamic_spring_weight = spring_weight_default;
        if (i > 1) {
            path[i].radius = std::max(static_cast<float>(min_rep_radius), std::min(static_cast<float>(max_rep_radius), distanceToClosestObstacle(path[i], max_rep_radius)));
        } else {
            path[i].radius = min_rep_radius;
        }

        float smooth_factor = 0.70f + 0.6f / (1.0f + exp(-10 * (2 * (path[i].radius - min_rep_radius) / (max_rep_radius - min_rep_radius) - 1)));
        dynamic_spring_weight *= smooth_factor;

        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        // dynamicSpringWeightTimes.push_back(duration);

        Point springForce = computeSpringForce(i, dynamic_spring_weight, dynamic_spring_radius);

        // start_time = std::chrono::high_resolution_clock::now();

        Point repulsiveForce = computeRepulsiveForce(i, repulsive_strength);

        // end_time = std::chrono::high_resolution_clock::now();
        // duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        // computeRepulsiveForceTimes.push_back(duration);

        if (i > 1) {
            bool is_corridor       = false;
            int corridor_neigbours = checkAndAjustInCorridor(i, small_gap_radius);
            if (corridor_neigbours > 3) {
                is_corridor = true;
                springForce.x *= 0.8;
                springForce.y *= 0.8;
                float corridor_scaling_factor = std::max(1.0f / static_cast<float>(corridor_neigbours), 0.2f);
                repulsiveForce.x *= corridor_scaling_factor;
                repulsiveForce.y *= corridor_scaling_factor;
                path[i].colour = cv::Scalar(255, 0, 0);
            } else {
                path[i].colour = cv::Scalar(0, 0, 200);
            }
        }

        Point displacement = { alpha * (springForce.x + repulsiveForce.x), alpha * (springForce.y + repulsiveForce.y) };
        Point newPos       = { path[i].x + displacement.x, path[i].y + displacement.y };

        if (maze.isFree(newPos)) {
            total_change = std::max(total_change, std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
            path[i]      = newPos;
        } else {
            float random_factor = 1.0 - repel_variation + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * repel_variation)));
            path[i].radius      = repel_raduis * random_factor;
            repelFromObstacle(i, 0.35);
        }
    }


    // auto average = [](const std::vector<long long>& times) { return times.size() > 0 ? std::accumulate(times.begin(), times.end(), 0LL) / times.size() : 0LL; };

    // std::cout << "Average point radius time : " << average(dynamicSpringWeightTimes) << " micro s, total for " << path.size()
    //           << " iterations: " << std::accumulate(dynamicSpringWeightTimes.begin(), dynamicSpringWeightTimes.end(), 0LL) << std::endl;

    // std::cout << "Average compRepuForce time: " << average(computeRepulsiveForceTimes) << " micro s, total for " << path.size()
    //           << "  iterations: " << std::accumulate(computeRepulsiveForceTimes.begin(), computeRepulsiveForceTimes.end(), 0LL) << std::endl;

    // total_radius_time += std::accumulate(dynamicSpringWeightTimes.begin(), dynamicSpringWeightTimes.end(), 0LL);
    // total_repulsive_time += std::accumulate(computeRepulsiveForceTimes.begin(), computeRepulsiveForceTimes.end(), 0LL);

    // dynamicSpringWeightTimes.clear();
    // computeRepulsiveForceTimes.clear();

    // showPath(show_time);


    current_iteration++;
    if (current_iteration >= max_iterations || total_change <= total_change_threshold) {
        optimization_complete = true;
    }

    return optimization_complete;

    // int key = cv::waitKey(2);
    // if (key >= 0) {
    //     if (key == 83) { // Right arrow key
    //         if (show_time < 10)
    //             show_time += 2;
    //         else
    //             show_time += static_cast<int>(2 + 0.3 * show_time);
    //         std::cout << "\n show_time increased to: " << show_time << std::endl;

    //     } else if (key == 81) { // Left arrow key
    //         if (show_time < 10 && show_time > 0)
    //             show_time -= 3;
    //         else
    //             show_time -= static_cast<int>(2 + 0.3 * show_time);
    //         if (show_time < 0)
    //             show_time = 0;
    //         std::cout << "\n show_time decreased to: " << show_time << std::endl;

    //     } else if (key == 32) {
    //         std::cout << "\nSpacebar was pressed, Pause..." << std::endl;
    //         while (true) {
    //             // showPath(show_time);
    //             int inner_key = cv::waitKey(1);
    //             if (inner_key == 27) { // Press 'ESC' to exit
    //                 std ::cout << "ESC from pause" << std::endl;
    //                 return inner_key;
    //             }
    //             if (inner_key != -1 && inner_key != 32) { // If a key other than spacebar is pressed
    //                 break;
    //             }
    //         }
    //         std::cout << "Pause ended" << std::endl;

    //     } else {
    //         std::cout << "\n Optimization interrupted by user." << std::endl;
    //         return 0;
    //     }
    // }


    // if (iter % 5 == 0) {
    //     std ::cout << ", show_time: " << show_time;
    //     std::cout << ", total_change: " << total_change;
    //     std::cout << std::endl;
    // }

    // if (total_change <= total_change_threshold) {
    //     // std::cout << "Optimization converged after " << iter << " iterations, becuase: total_change vs total_change_threshold: " << total_change << " vs "
    //     //           << total_change_threshold << std::endl;
    //     break;
    // }
    // }

    std ::cout << "Total time for radius calculation: " << (float)total_radius_time / 1000 << " ms" << std::endl;
    std ::cout << "Total time for repulsive force calculation: " << (float)total_repulsive_time / 1000 << " ms" << std::endl;

    if (total_change > total_change_threshold) {
        // std::cout << "Optimization ended because of max. iterations: " << max_iterations << std::endl;
    }


    return 1;
}

int ElasticBand::checkAndAjustInCorridor(size_t idx, int range) const
{
    // Check if the point is in a corridor and valid_neighbors the number of neighbors
    int corridor_neigbours = 0;
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            if (dx == 0 && dy == 0)
                continue; // Skip the current point
            if (!maze.isFree(path[idx].x + dx, path[idx].y + dy) && !maze.isFree(path[idx].x - dx, path[idx].y - dy)) {
                corridor_neigbours++;
            }
        }
    }
    return corridor_neigbours;
}

Point ElasticBand::computeSpringForce(size_t idx, const float spr_weight, int radius) const
{
    const Point& current = path[idx];
    Point force          = { 0, 0 };
    int valid_neighbors = 0; // used to calculate the number of neighbors contributing to the force: so that spring force pulls the current point toward the average Point of its neighbors

    // Iterate over the points within the radius
    for (int i = -radius; i <= radius; ++i) {
        if (i == 0)
            continue; // Skip the current point

        int neighbor_idx = idx + i;
        if (neighbor_idx >= 0 && neighbor_idx < path.size()) {
            Point neighbor = path[neighbor_idx];
            force.x += neighbor.x;
            force.y += neighbor.y;
            valid_neighbors++;
        }
    }

    if (valid_neighbors > 0) {
        force.x = (force.x / valid_neighbors - current.x) * spr_weight; //(average position of neighbors - current position) * spring weight
        force.y = (force.y / valid_neighbors - current.y) * spr_weight;
    }

    return force;
}

Point ElasticBand::computeRepulsiveForce(size_t idx, const float rep_strength) const
{
    const Point& current = path[idx];
    Point force          = { 0, 0 };

    for (int dx = -current.radius; dx <= current.radius; ++dx) {
        for (int dy = -current.radius; dy <= current.radius; ++dy) {
            float distance = std::hypot(dx, dy);

            if (distance > 0 && distance <= current.radius) {
                Point neighbor = { current.x + dx, current.y + dy };

                if (!maze.isFree(neighbor)) {
                    float scale = rep_strength / (distance * distance);

                    force.x -= dx * scale;
                    force.y -= dy * scale;
                }
            }
        }
    }

    // Remove tangential component
    if (idx > 0 && idx < path.size() - 1) {
        Point tangent      = { path[idx - 1].x - path[idx + 1].x, path[idx - 1].y - path[idx + 1].y };
        float tangent_norm = std::hypot(tangent.x, tangent.y);

        if (tangent_norm > 0) {
            tangent.x /= tangent_norm;
            tangent.y /= tangent_norm;

            float dot_product = force.x * tangent.x + force.y * tangent.y;

            // Subtract tangential component
            force.x -= dot_product * tangent.x;
            force.y -= dot_product * tangent.y;
        }
    }

    // // Get the distance from the current point to the nearest obstacle
    // Point force2 = { 0, 0 };
    // float distance = maze.getDistanceToObstacle(current);

    // // Proceed only if the point is within the influence radius
    // if (distance > 0 && distance <= current.radius) {
    //     // Compute the gradient of the distance field using central differences
    //     int x = static_cast<int>(std::round(current.x));
    //     int y = static_cast<int>(std::round(current.y));

    //     // Gradient computation
    //     float dx = maze.getDistanceToObstacle(Point(x + 1, y)) - maze.getDistanceToObstacle(Point(x - 1, y));
    //     float dy = maze.getDistanceToObstacle(Point(x, y + 1)) - maze.getDistanceToObstacle(Point(x, y - 1));
    //     // Add diagonal components to the gradient
    //     float ddx = (maze.getDistanceToObstacle(Point(x + 1, y + 1)) - maze.getDistanceToObstacle(Point(x - 1, y - 1))) / std::sqrt(2.0f);
    //     float ddy = (maze.getDistanceToObstacle(Point(x + 1, y - 1)) - maze.getDistanceToObstacle(Point(x - 1, y + 1))) / std::sqrt(2.0f);

    //     dx += ddx;
    //     dy += ddy;

    //     // Normalize the gradient to obtain the direction of the repulsive force
    //     float norm = std::hypot(dx, dy);
    //     if (norm > 0) {
    //         dx /= norm;
    //         dy /= norm;

    //         // Scale the force by the inverse square of the distance
    //         float epsilon = 0.01f;                       // Small value to prevent division by zero
    //         distance      = std::max(distance, epsilon); // Avoid zero distance
    //         float scale   = rep_strength / (distance * distance);

    //         // Apply scaled gradient to compute force
    //         force2.x = -scale * dx;
    //         force2.y = -scale * dy;
    //     }
    // }


    // std ::cout << "Force(for): " << force.x << " " << force.y << " Force2(disttransf.): " << force2.x << " " << force2.y << std::endl;

    return force;
}

void ElasticBand::repelFromObstacle(size_t idx, float rep_strength)
{
    Point& current = path[idx];
    Point force    = { 0, 0 };

    for (int dx = -current.radius; dx <= current.radius; ++dx) {
        for (int dy = -current.radius; dy <= current.radius; ++dy) {
            float distance = std::hypot(dx, dy);

            if (distance > 0 && distance <= current.radius) {
                Point neighbor = { current.x + dx, current.y + dy };

                if (!maze.isFree(neighbor)) {
                    float scale = rep_strength / (distance * distance);

                    force.x -= dx * scale;
                    force.y -= dy * scale;
                }
            }
        }
    }

    for (int i = 0; i < 10; ++i) {
        current.x += force.x;
        current.y += force.y;
        if (maze.isFree(current)) {
            break;
        }
    }
}

const std::vector<Point>& ElasticBand::getPath() const { return path; }

} // namespace ecn
