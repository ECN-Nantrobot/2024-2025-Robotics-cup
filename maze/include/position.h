// position.h
#ifndef POSITION_H
#define POSITION_H

#include <vector>
#include <cmath>
#include <iostream>
#include <maze.h>
#include <point.h>
#include <position.h> // Include the new header file


using namespace std;
using namespace ecn;

class Position : public Point
{
public:
    Position() : Point(0, 0, 0) {} // Default to (0, 0) with no additional cost

    // Constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // Constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    // Constructor with distance
    Position(int _x, int _y, float _dist) : Point(_x, _y, _dist) {}

    // Position_float(float _x, float _y, float _dist) : Point(_x, _y, _dist) {}



    double distToParent(){
        int penalty = 0;
        return Position::dist + penalty;
    }

    std::vector<Position> children()
    {
        std::vector<Position> generated(8);
        int i = 0;
        auto x = Point::x;
        auto y = Point::y;

        if (maze.isFree(x + 1, y)) generated[i++] = Position(x + 1, y, 1);
        if (maze.isFree(x, y + 1)) generated[i++] = Position(x, y + 1, 1);
        if (maze.isFree(x, y - 1)) generated[i++] = Position(x, y - 1, 1);
        if (maze.isFree(x - 1, y)) generated[i++] = Position(x - 1, y, 1);

        if (maze.isFree(x + 1, y + 1)) generated[i++] = Position(x + 1, y + 1, std::sqrt(2));
        if (maze.isFree(x - 1, y + 1)) generated[i++] = Position(x - 1, y + 1, std::sqrt(2));
        if (maze.isFree(x + 1, y - 1)) generated[i++] = Position(x + 1, y - 1, std::sqrt(2));
        if (maze.isFree(x - 1, y - 1)) generated[i++] = Position(x - 1, y - 1, std::sqrt(2));

        auto goal = maze.end();

        // Check if any child reaches the goal
        for (int j = 0; j < i; ++j){

            if (generated[j].x == goal.x && generated[j].y == goal.y){

                std::cout << "GOAL reached at (" << generated[j].x << ", " << generated[j].y << ")" << std::endl;
                return {generated[j]}; // Return only the goal position
            }
        }

        return std::vector<Position>(generated.begin(), generated.begin() + i); // Only return valid positions
    }

    // Define equality operator
    bool operator==(const Position& other) const
    {
        return x == other.x && y == other.y;
    }

    // Define inequality operator
    bool operator!=(const Position& other) const
    {
        return !(*this == other);
    }
    
};

#endif // POSITION_H