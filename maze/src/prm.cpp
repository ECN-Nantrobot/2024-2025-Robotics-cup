#include <point.h>
#include "point_hash.h"
#include <prm.h>
#include <random>
// #include <unordered_map>


namespace ecn
{

std::vector<Point> generateRandomPoints(int numPoints, int width, int height, const Maze& maze)
{
    std::vector<Point> points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> xDist(0, width - 1);
    std::uniform_int_distribution<> yDist(0, height - 1);

    while (points.size() < numPoints) {
        int x = xDist(gen);
        int y = yDist(gen);
        if (maze.isFree(x, y)) {
            points.emplace_back(x, y);
        }
    }

    return points;
}

std::vector<std::pair<Point, Point>> connectPoints(const std::vector<Point>& points, float distanceThreshold, const Maze& maze)
{
    std::vector<std::pair<Point, Point>> edges;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            float distance = std::sqrt(std::pow(points[i].x - points[j].x, 2) + std::pow(points[i].y - points[j].y, 2));
            if (distance <= distanceThreshold && maze.isLineFree(points[i], points[j])) {
                edges.emplace_back(points[i], points[j]);
            }
        }
    }
    return edges;
}

std::unordered_map<Point, std::vector<std::pair<Point, float>>> buildGraph(const std::vector<Point>& points, const std::vector<std::pair<Point, Point>>& edges)
{
    std::unordered_map<Point, std::vector<std::pair<Point, float>>> graph;
    for (const auto& edge : edges) {
        float distance = std::sqrt(std::pow(edge.first.x - edge.second.x, 2) + std::pow(edge.first.y - edge.second.y, 2));
        graph[edge.first].emplace_back(edge.second, distance);
        graph[edge.second].emplace_back(edge.first, distance);
    }
    return graph;
}

void connectStartAndGoal(const Point& start, const Point& goal, std::unordered_map<Point, std::vector<std::pair<Point, float>>>& graph, const std::vector<Point>& points, float distanceThreshold, const Maze& maze)
{
    for (const auto& point : points) {
        float startDist = std::sqrt(std::pow(start.x - point.x, 2) + std::pow(start.y - point.y, 2));
        float goalDist  = std::sqrt(std::pow(goal.x - point.x, 2) + std::pow(goal.y - point.y, 2));

        if (startDist <= distanceThreshold && maze.isLineFree(start, point)) {
            graph[start].emplace_back(point, startDist);
            graph[point].emplace_back(start, startDist);
        }

        if (goalDist <= distanceThreshold && maze.isLineFree(goal, point)) {
            graph[goal].emplace_back(point, goalDist);
            graph[point].emplace_back(goal, goalDist);
        }
    }
}

} // namespace ecn
