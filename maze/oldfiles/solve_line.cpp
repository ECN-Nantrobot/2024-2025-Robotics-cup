#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
public:


    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    // constructor from coordinates
    Position(int _x, int _y, int _dist) : Point(_x, _y, _dist) {}



    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        //std::cout << "Distance to parent: " << Point::dist << std::endl;
        return Point::dist;
    }

    // bool is_corridor(int, int)
    // {
    // }

    void searchInLine(int dx, int dy, std::vector<Position>& generated) {
        int current_x = x;
        int current_y = y;
        int current_d = 0;

        //while next one is free
        while (maze.isFree((current_x+dx), (current_y+dy))) {
            current_d = current_d + 1; //increment distance

            //check left and right
            if(dx != 0)
            {
                current_x += dx; //go to the next free one
                if(maze.isFree(current_x, (current_y+1)) or maze.isFree(current_x, (current_y-1)))
                    break;
            }

            if(dy != 0)
            {
                current_y += dy; //go to the next free one
                if(maze.isFree((current_x+1), current_y) or maze.isFree((current_x-1), current_y))
                    break;
            }
        }
        //std::cout << "Child x, y, d:    " << current_x << ", " << current_y << ", " << current_d << std::endl;

        generated.push_back(Position(current_x, current_y, current_d));
    }

    std::vector<Position> children() {
        std::vector<Position> generated;
        // auto x = Point::x;
        // auto y = Point::y;

        //std::cout << "Current x, y:      " << x << ", " << y << std::endl;
        // Search in all four directions
        //std::cout << "Searching in +x:\n";
        searchInLine(1, 0, generated);   // +x direction
        //std::cout << "Searching in -x:\n";
        searchInLine(-1, 0, generated);  // -x direction
        //std::cout << "Searching in +y:\n";
        searchInLine(0, 1, generated);   // +y direction
        //std::cout << "Searching in -y:\n";
        searchInLine(0, -1, generated);  // -y direction

        return generated;
    }



//     std::vector<Position> children()
//     {
//         // this method should return  all positions reachable from this one
//         std::vector<Position> generated;

//         auto x = Point::x;
//         auto y = Point::y;
//         auto d = distance;

//         // TODO add free reachable positions from this point

//         std::vector<Position> searchInLine(Maze& maze, int dir, std::vector<Position>& generated)
//         {
//             while(maze.isFree(x, y))
//             {
//                 switch (dir) {
//                 case 1:
//                     x++;
//                     break;
//                 case 2:
//                     x--;
//                     break;
//                 case 3:
//                     y++;
//                     break;
//                 case 4:
//                     y--;
//                     break;
//                 default:
//                     break;
//                 }

//                 d++;
//             }
//             generated.push_back(Position(x, y, d));
//             x = 0;
//             y = 0;
//             d = 0;
//         }

//         searchInLine(maze,1, generated);
//         searchInLine(maze,2, generated);
//         searchInLine(maze,3, generated);
//         searchInLine(maze,4, generated);

//         return generated;
//     }
};


int main( int argc, char **argv )
{
    // load file
    std::string filename = Maze::mazeFile("maze.png");
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    std::cout << "searching ..." << std::endl;

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("cell");
    cv::waitKey(0);
}
