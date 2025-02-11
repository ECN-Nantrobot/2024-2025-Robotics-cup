#ifndef PRM_H
#define PRM_H

#include "point.h"       // Definition of ecn::Point
#include "maze.h"        // Definition of ecn::Maze
#include <unordered_map>
#include <vector>

namespace ecn
{

// Function to generate random points within the maze boundaries
std::vector<Point> generateRandomPoints(int numPoints, int width, int height, const Maze& maze);

// Function to connect points that are within a certain distance threshold
std::vector<std::pair<Point, Point>> connectPoints(const std::vector<Point>& points, float distanceThreshold, const Maze& maze);

// Function to build a graph from points and their connections
std::unordered_map<Point, std::vector<std::pair<Point, float>>> buildGraph(
    const std::vector<Point>& points,
    const std::vector<std::pair<Point, Point>>& edges);

// Function to connect start and goal points to the graph
void connectStartAndGoal(
    const Point& start,
    const Point& goal,
    std::unordered_map<Point, std::vector<std::pair<Point, float>>>& graph,
    const std::vector<Point>& points,
    float distanceThreshold,
    const Maze& maze);

} // namespace ecn

#endif // PRM_H
