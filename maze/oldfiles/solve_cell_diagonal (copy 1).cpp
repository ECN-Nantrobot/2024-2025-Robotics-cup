#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is an x-y position, we move by 1 each time
class Position : public Point
{
public:
    // Constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // Constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    // Constructor with distance
    Position(int _x, int _y, float _dist) : Point(_x, _y, _dist) {}

    // Returns the distance to the parent node
    double distToParent()
    {
        int penalty = 0;

        // Check adjacent cells for obstacles
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0) continue; // Skip current cell

                int nx = x + dx;
                int ny = y + dy;

                if (!maze.isFree(nx, ny))
                {
                    penalty += 10; // Add a penalty for each adjacent obstacle
                }
            }
        }

        // Base cost + penalty
        return Position::dist + penalty;

//        return Position::dist;
    }

    // Generate reachable positions from the current position
    std::vector<Position> children()
    {
        std::vector<Position> generated;
        auto x = Point::x;
        auto y = Point::y;

        // Check adjacent cells
        if (maze.isFree(x + 1, y))
        {
            generated.push_back(Position(x + 1, y, 1));
        }
        if (maze.isFree(x, y + 1))
        {
            generated.push_back(Position(x, y + 1, 1));
        }
        if (maze.isFree(x, y - 1))
        {
            generated.push_back(Position(x, y - 1, 1));
        }
        if (maze.isFree(x - 1, y))
        {
            generated.push_back(Position(x - 1, y, 1));
        }

        // Check diagonal cells
        if (maze.isFree(x + 1, y + 1)) // Bottom-right
        {
            generated.push_back(Position(x + 1, y + 1, std::sqrt(2)));
        }
        if (maze.isFree(x - 1, y + 1)) // Bottom-left
        {
            generated.push_back(Position(x - 1, y + 1, std::sqrt(2)));
        }
        if (maze.isFree(x + 1, y - 1)) // Top-right
        {
            generated.push_back(Position(x + 1, y - 1, std::sqrt(2)));
        }
        if (maze.isFree(x - 1, y - 1)) // Top-left
        {
            generated.push_back(Position(x - 1, y - 1, std::sqrt(2)));
        }

        auto goal = maze.end(); // Goal position
            for (const auto &child : generated)
            {
                if (child.x == goal.x && child.y == goal.y)
                {
                    std::cout << "GOOOAAAAL reached at (" << child.x << ", " << child.y << ")" << std::endl;
                    generated.clear();
                    generated.push_back(child);
                    break;
                }
            }

        return generated;
    }
};

int main(int argc, char **argv)
{
    // Load maze file
    std::string filename = Maze::mazeFile("maze.png");
    if (argc == 2)
        filename = std::string(argv[1]);

    std::cout << "Loading maze from file: " << filename << std::endl;

    // Let Point know about this maze
    Position::maze.load(filename);

    // Debugging message for successful load
    std::cout << "Maze loaded successfully. Ensure it is visually checked." << std::endl;

    // Get initial and goal positions
    Position start = Position::maze.start(), goal = Position::maze.end();

    // Debugging output for start and goal positions
    std::cout << "Start position: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal.x << ", " << goal.y << ")" << std::endl;

    std::cout << "Starting A* search..." << std::endl;

    // Call A* algorithm
    ecn::Astar(start, goal);

    // Save the solution
    std::cout << "Search completed. Saving solution..." << std::endl;
    Position::maze.saveSolution("cell"); // No return value, so no condition here
    std::cout << "Solution saved successfully to 'cell.png'" << std::endl;

    // Wait for user to view the result
    cv::waitKey(0);

    return 0;
}
