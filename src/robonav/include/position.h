// position.h
#ifndef POSITION_H
#define POSITION_H

#include <cmath>
#include <iostream>
#include <maze.h>
#include <point.h>
#include <vector>


namespace ecn
{

class Position : public Point
{
public:
    float dist;
    Position() : Point(0, 0) {} // Default to (0, 0) with no additional cost

    // Constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // Constructor from base ecn::Point
    Position(Point p) : Point(p.x, p.y) {}

    // Constructor with distance
    Position(int _x, int _y, float _dist) : Point(_x, _y), dist(_dist) {}


    double distToParent() { return dist; }

    std::vector<Position> children(const Position& parent)
    // std::vector<Position> children()
    {
        std::vector<Position> generated(4);
        int i = 0;
        int x = Point::x;
        int y = Point::y;

        // doesnt generate nodes/children that are/lead back to the parent
        if (Point::maze.isFree(x + 1, y, true) && !(parent.x == x + 1 && parent.y == y)) 
            generated[i++] = Position(x + 1, y, 1);
        if (Point::maze.isFree(x, y + 1, true) && !(parent.x == x && parent.y == y + 1))
            generated[i++] = Position(x, y + 1, 1);
        if (Point::maze.isFree(x, y - 1, true) && !(parent.x == x && parent.y == y - 1))
            generated[i++] = Position(x, y - 1, 1);
        if (Point::maze.isFree(x - 1, y, true) && !(parent.x == x - 1 && parent.y == y))
            generated[i++] = Position(x - 1, y, 1);

        auto goal = Point::maze.getGoal();

        // Check if any child is the goal
        for (int j = 0; j < i; ++j)
        {
            if (generated[j].x == goal.x && generated[j].y == goal.y)
            {
                std::cout << "GOAL reached at (" << generated[j].x << ", " << generated[j].y << ")" << std::endl;
                return { generated[j] }; // Return only goal position
            }
        }

        return std::vector<Position>(generated.begin(), generated.begin() + i); // Only return valid positions
    }

    bool operator==(const Position& other) const { return x == other.x && y == other.y; }

    bool operator!=(const Position& other) const { return !(*this == other); }

    // Overload the * operator for scalar multiplication
    Position operator*(float scalar) const {
        return Position(static_cast<int>(x * scalar), static_cast<int>(y * scalar));
    }
};
} // namespace ecn
#endif // POSITION_H