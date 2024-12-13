#include "elastic_bands.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <maze.h>
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

void ElasticBand::fillGaps(int max_gap)
{
    std::vector<Point> filledPath;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        Point current = path[i];
        Point next    = path[i + 1];

        filledPath.push_back(current); // Keep the current point

        // Compute the distance to the next point
        float distance = std::hypot(next.x - current.x, next.y - current.y);

        // Add intermediate points if the gap is too large
        if (distance > max_gap) {
            int num_of_interm_points = static_cast<int>(std::ceil(distance / max_gap)) - 1;
            for (int j = 1; j <= num_of_interm_points; ++j) {
                Point intermediate = { current.x + (float)j * (next.x - current.x) / (num_of_interm_points + 1), current.y + (float)j * (next.y - current.y) / (num_of_interm_points + 1) };

                // Only add the point if it's free
                if (maze.isFree(intermediate)) {
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

std::vector<Point> ElasticBand::gaussianSmoothing(const std::vector<Point>& path, int windowSize, float sigma)
{
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

    // Apply Gaussian smoothing
    for (size_t i = 0; i < path.size(); ++i) {
        float sumX = 0.0f, sumY = 0.0f;

        // Apply the kernel to the neighbors in the window
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < path.size()) {
                sumX += path[idx].x * kernel[j + halfWindow];
                sumY += path[idx].y * kernel[j + halfWindow];
            }
        }

        smoothedPath.push_back(Point{ sumX, sumY });
    }

    return smoothedPath;
}

// Function to generate a path by filling gaps and smoothing it
void ElasticBand::generateSmoothedPath(const std::vector<Point>& path, float maxGap, int windowSize, float sigma)
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
    const int grid_size = 1;  // Grid line thickness

    // Create a higher-resolution blank image
    cv::Mat visualization;
    cv::resize(maze.getIm(), visualization, cv::Size(), scale, scale, cv::INTER_NEAREST);

    // Draw the points
    for (const auto& point : path) {
        cv::rectangle(visualization, cv::Point(point.x * scale, point.y * scale), cv::Point((point.x + 1) * scale - grid_size, (point.y + 1) * scale - grid_size), point.colour, cv::FILLED);
    }

    // Draw Circles aroung the points
    for (const auto& point : path) {
        cv::circle(visualization, cv::Point(point.x * scale + scale / 2, point.y * scale + scale / 2),
                   point.radius * scale, // Adjust radius based on scale
                   point.colour,
                   scale / 5); // Thickness proportional to scale
    }

    std::string window_name = "Elastic Band Optimization, resolution " + std::to_string(scale) + ":1";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1500, 1200);
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
    float min_distance = search_radius;

    // Iterate over the local area around the point
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            int nx = point.x + dx;
            int ny = point.y + dy;

            if (nx < 0 || nx >= maze.width() || ny < 0 || ny >= maze.height()) {
                continue;
            }

            if (!maze.isFree(nx, ny)) {
                float distance = std::hypot(dx, dy);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }
    }

    return min_distance == std::numeric_limits<float>::max() ? -1.0f : min_distance; // Return -1 if no obstacle found
}

bool ElasticBand::resizePath(float min_dist, float max_dist)
{
    int max_path_size = 1000;
    std::vector<Point> adjustedPath(max_path_size * 1.4);
    size_t i_adjusted = 0;
    int n             = 0;

    adjustedPath[i_adjusted++] = path.front();

    for (size_t i = 1; i < path.size(); ++i) {
        Point previous = path[i - 1];
        Point current  = path[i + n];

        float distance = std::hypot(current.x - previous.x, current.y - previous.y);
        // std::cout << "Distance from " << i -1 << " to " << i+n << " = " << distance << " ";


        // if (i < 3 || i > path.size() - 2) {
        //     adjustedPath[i_adjusted++] = current; // increment after assignement
        //     continue;
        // } else {

        if (distance >= min_dist && distance <= max_dist) {
            adjustedPath[i_adjusted++] = current; // increment after assignement

            // std::cout << "added: " <<  i + n << " to " << i_adjusted -1 << std::endl;

            if (i + n < path.size() - 1)
                i = i + n;
            else
                break;
            n = 0;
        }

        else if (distance > max_dist) {
            int num_of_interm_points = std::min(static_cast<int>(std::ceil(distance / max_dist)) - 1, 5);
            // std ::cout << "num_of_interm_points: " << num_of_interm_points << std::endl;

            for (int j = 1; j <= num_of_interm_points; ++j) {
                Point intermediate         = { previous.x + (float)j * (current.x - previous.x) / (num_of_interm_points + 1),
                                               previous.y + (float)j * (current.y - previous.y) / (num_of_interm_points + 1) };
                adjustedPath[i_adjusted++] = intermediate;
                // std::cout << "added intermdiate: " << i_adjusted -1 << std::endl;
            }
            n = 0;
        } else {
            i--;
            if (i + n + 1 < path.size() - 1)
                n++;
            else
                break;
        }
        // }
        // std::cout << "i: " << i << ", " << n << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // std::cout << "adjusted index: " << i_adjusted << std::endl;
    }

    // std::cout << "Old Path:" << std::endl;
    // for (size_t i = 1; i < path.size(); ++i)
    // {
    //     float distance = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    //     std::cout << "Index: " << i << ", Distance to previous point: " << distance << std::endl;
    // }

    // // Print index and distance to the point before for the new path
    // std::cout << "New Path:" << std::endl;
    // for (size_t i = 1; i < adjustedPath.size(); ++i)
    // {
    //     float distance = std::hypot(adjustedPath[i].x - adjustedPath[i - 1].x, adjustedPath[i].y - adjustedPath[i - 1].y);
    //     std::cout << "Index: " << i << ", Distance to previous point: " << distance << std::endl;
    // }

    adjustedPath[i_adjusted++] = path.back();
    std ::cout << "Path.size: " << path.size();

    adjustedPath.resize(i_adjusted);
    path = adjustedPath;
    if (path.size() > max_path_size) {
        return false;
    }
    std ::cout << " -> " << path.size();
    return true;
}


int ElasticBand::optimize()
{
    static int show_time = 30;

    const float alpha                  = 0.077; // Step size (scaling of the total force)
    const int max_iterations           = 40;
    const float total_change_threshold = 0.0003; //(total distanc of movement of points)
    float total_change                 = 0;

    const float spring_weight_default = 16.5;
    const int spring_radius           = 6; // Radius of the spring force (average Point of neighbors in radius)
    int dynamic_spring_radius         = 6;
    float rep_to_spring_radius_factor = 0.4;

    const float repulsive_strength = 6.0;
    const int min_rep_radius       = 5;
    const int max_rep_radius       = 18;
    float dynamic_rep_radius       = min_rep_radius;
    const int repel_raduis         = 24;
    const float repel_variation    = 0.3; // +- 40%

    const int small_gap_radius = 5;

    float min_distance = 2.0;
    float max_distance = 3.3;


    for (int iter = 0; iter < max_iterations; ++iter) {

        std::cout << "Iter: " << iter << ": ";
        total_change = 0;

        if (!resizePath(min_distance, max_distance)) {
            std::cerr << "Optimization exited early: resizePath too long" << std::endl;
            return 100;
        }


        // Iterate over all points except start and end
        for (size_t i = 1; i < path.size() - 1; ++i) {

            float dynamic_spring_weight = spring_weight_default;
            path[i].radius = std::max(static_cast<float>(min_rep_radius), std::min(static_cast<float>(max_rep_radius), distanceToClosestObstacle(path[i], max_rep_radius)));
            dynamic_spring_radius = path[i].radius * rep_to_spring_radius_factor;

            // Calculates a smooth_factor based on the radius, which lies within the range [min_rep_radius, max_rep_radius].
            // The factor is scaled within the range [0.8, 1.2], with the value for radius min_rep_radius near 0.8 and for radius max_rep_radius near 1.2.
            float smooth_factor = 0.85f + 0.3f / (1.0f + exp(-10 * (2 * (path[i].radius - min_rep_radius) / (max_rep_radius - min_rep_radius) - 1)));
            dynamic_spring_weight *= smooth_factor;

            Point springForce    = computeSpringForce(i, dynamic_spring_weight, dynamic_spring_radius);
            Point repulsiveForce = computeRepulsiveForce(i, repulsive_strength);

            // Reduce forces if in a corridor
            bool is_corridor       = false;
            int corridor_neigbours = checkAndAjustInCorridor(i, small_gap_radius);
            if (corridor_neigbours > 3) {
                is_corridor = true;
                springForce.x *= 0.8;
                springForce.y *= 0.8;
                float corridor_scaling_factor = std::max(1.0f / static_cast<float>(corridor_neigbours), 0.2f); // Minimum scale = 0.1
                repulsiveForce.x *= corridor_scaling_factor;
                repulsiveForce.y *= corridor_scaling_factor;
                path[i].colour = cv::Scalar(255, 0, 0); // blue
            } else {
                path[i].colour = cv::Scalar(0, 0, 255); // red
            }

            Point displacement = { alpha * (springForce.x + repulsiveForce.x), alpha * (springForce.y + repulsiveForce.y) };
            Point newPos       = { path[i].x + displacement.x, path[i].y + displacement.y };

            if (maze.isFree(newPos)) {
                total_change = std::max(total_change, std::hypot(newPos.x - path[i].x, newPos.y - path[i].y));
                path[i]      = newPos; // Update the path point to the new position
            } else {
                // Choose random values between -repel_variation% and +repel_variation% of repel_radius
                float random_factor = 1.0 - repel_variation + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * repel_variation)));
                path[i].radius      = repel_raduis * random_factor;
                repelFromObstacle(i, /*rep_strength=*/0.35);
            }
        }


        int key = cv::waitKey(2);
        if (key >= 0) {
            if (key == 83) { // Right arrow key
                if (show_time < 10)
                    show_time += 2;
                else
                    show_time += static_cast<int>(2 + 0.3 * show_time);
                std::cout << "\n show_time increased to: " << show_time << std::endl;

            } else if (key == 81) { // Left arrow key
                if (show_time < 10 && show_time > 0)
                    show_time -= 3;
                else
                    show_time -= static_cast<int>(2 + 0.3 * show_time);
                if (show_time < 0)
                    show_time = 0;
                std::cout << "\n show_time decreased to: " << show_time << std::endl;

            } else if (key == 32) {
                std::cout << "\nSpacebar was pressed, Pause..." << std::endl;
                while (true) {
                    showPath(show_time);
                    int inner_key = cv::waitKey(1);
                    if (inner_key == 27) { // Press 'ESC' to exit
                        std ::cout << "ESC from pause" << std::endl;
                        return inner_key;
                    }
                    if (inner_key != -1 && inner_key != 32) { // If a key other than spacebar is pressed
                        break;
                    }
                }
                std::cout << "Pause ended" << std::endl;

            } else {
                std::cout << "\n Optimization interrupted by user." << std::endl;
                return 0;
            }
        }

        showPath(show_time);

        std ::cout << ", show_time: " << show_time;
        std::cout << ", total_change: " << total_change;
        std::cout << std::endl;

        if (total_change <= total_change_threshold) {
            std::cout << "Optimization converged after " << iter << " iterations, becuase: total_change vs total_change_threshold: " << total_change << " vs "
                      << total_change_threshold << std::endl;
            break;
        }
    }

    if (total_change > total_change_threshold) {
        std::cout << "Optimization ended because of max. iterations: " << max_iterations << std::endl;
    }

    // fillGaps(1); // Fill gaps after optimization (maybe not needed later in real world)
    return 1;
}

int ElasticBand::checkAndAjustInCorridor(size_t idx, int range) const
{
    // Check if the point is in a corridor and count the number of neighbors
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
    int count = 0; // used to calculate the number of neighbors contributing to the force: so that spring force pulls the current point toward the average Point of its neighbors

    // Special handling for the first and last movable points (send and second last point)
    if (idx == 1) {
        force.x += (path[0].x - path[idx].x) * spr_weight; // Pull towards start
        force.y += (path[0].y - path[idx].y) * spr_weight;
    } else if (idx == path.size() - 2) {
        force.x += (path.back().x - path[idx].x) * spr_weight; // Pull towards end
        force.y += (path.back().y - path[idx].y) * spr_weight;
    } else {

        // Iterate over the points within the radius
        for (int i = -radius; i <= radius; ++i) {
            if (i == 0)
                continue; // Skip the current point

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

    // // Compute the norm (magnitude) of the force vector
    // float norm = std::hypot(force.x, force.y);

    // // Normalize the force vector if the norm is non-zero
    // if (norm > 0) {
    //     force.x /= norm;
    //     force.y /= norm;
    // }
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
