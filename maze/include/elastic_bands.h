#ifndef ELASTIC_BANDS_H
#define ELASTIC_BANDS_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <a_star.h>
#include <maze.h>
#include <position.h> // Include the new header file

using namespace std;
using namespace ecn;

class ElasticBand
{
public:
    ElasticBand(const vector<Position>& initial_path, const Maze& maze)
        : path(initial_path), maze(maze) {}

    void optimize()
    {
        bool optimized = false;
        while (!optimized)
        {
            optimized = true;
            for (size_t i = 1; i < path.size() - 1; ++i)
            {
                Position& prev = path[i - 1];
                Position& curr = path[i];
                Position& next = path[i + 1];

                Position new_pos = adjustNode(prev, curr, next);
                if (new_pos != curr)
                {
                    curr = new_pos;
                    optimized = false;
                }
                new_pos.drawCircle(maze.im, cv::Scalar(0, 255, 0));
            }
        }
    }

    const vector<Position>& getPath() const { return path; }

    void drawCircles(cv::Mat &image, const cv::Scalar &color, int radius = 5) const
    {
        for (const auto& pos : path)
        {
            pos.drawCircle(image, color, radius);
        }
    }

private:
    vector<Position> path;
    const Maze& maze;

    Position adjustNode(const Position& prev, const Position& curr, const Position& next)
    {
        // Calculate the new position based on the elastic band algorithm
        float alpha = 0.5; // Weight for the current position
        float beta = 0.25; // Weight for the previous and next positions

        int new_x = static_cast<int>(alpha * curr.x + beta * (prev.x + next.x));
        int new_y = static_cast<int>(alpha * curr.y + beta * (prev.y + next.y));

        Position new_pos(new_x, new_y);

        // Ensure the new position is free of obstacles
        if (maze.isFree(new_pos.x, new_pos.y))
        {
            return new_pos;
        }
        return curr;
    }
};

#endif // ELASTIC_BANDS_H