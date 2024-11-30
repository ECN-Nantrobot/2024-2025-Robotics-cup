#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
public:

    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from coordinates
    Position(int _x, int _y, int _distance) : Point(_x, _y, _distance) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        //std::cout << "Distance to parent: " << Position::distance << std::endl;
        return Position::dist;
    }

    bool is_corridor(int x, int y) const // if on both opposide sides or on 2 sides next to each other (i.e.: up & right) are walls
    {
        if((!maze.isFree(x+1, y) && !maze.isFree(x-1, y)) || (!maze.isFree(x, y+1) && !maze.isFree(x, y-1))
                                    || (!maze.isFree(x+1, y) && !maze.isFree(x, y+1))
                                    || (!maze.isFree(x+1, y) && !maze.isFree(x, y-1))
                                    || (!maze.isFree(x-1, y) && !maze.isFree(x, y+1))
                                    || (!maze.isFree(x-1, y) && !maze.isFree(x, y-1))){
            return true;
        }

        else{
            return false;
        }
    }

    enum direc {
        UP,
        DOWN,
        LEFT,
        RIGHT
    };

    const char* direcToString(direc direction) {
        switch (direction) {
            case UP:    return "UP";
            case DOWN:  return "DOWN";
            case LEFT:  return "LEFT";
            case RIGHT: return "RIGHT";
            default:    return "UNKNOWN";
        }
    }

    bool rightIsFree(int curr_x, int curr_y, direc direction) const
    {
        switch (direction) {
            case UP:
                if (maze.isFree(curr_x + 1, curr_y))
                    return true;
                break;
            case DOWN:
                if (maze.isFree(curr_x - 1, curr_y))
                    return true;
                break;
            case LEFT:
                if (maze.isFree(curr_x, curr_y - 1))
                    return true;
                break;
            case RIGHT:
                if (maze.isFree(curr_x, curr_y + 1))
                    return true;
                break;
            default:
                break;
        }
        return false;
    }

    bool leftIsFree(int curr_x, int curr_y, direc direction) const
    {
       switch (direction) {
           case UP:
               if (maze.isFree(curr_x - 1, curr_y))
                   return true;
               break;
           case DOWN:
               if (maze.isFree(curr_x + 1, curr_y))
                   return true;
               break;
           case LEFT:
               if (maze.isFree(curr_x, curr_y + 1))
                   return true;
               break;
           case RIGHT:
               if (maze.isFree(curr_x, curr_y - 1))
                   return true;
               break;
           default:
               std::cout << "Invalid direction!" << std::endl;
               break;
       }
       return false;
    }

    direc turnRight(direc direction) const {
        switch (direction) {
            case UP:
                return RIGHT;
            case RIGHT:
                return DOWN;
            case DOWN:
                return LEFT;
            case LEFT:
                return UP;
            default:
                std::cout << "Invalid direction!" << std::endl;
                return direction;
        }
    }

    direc turnLeft(direc direction) const {
        switch (direction) {
            case UP:
                return LEFT;
            case LEFT:
                return DOWN;
            case DOWN:
                return RIGHT;
            case RIGHT:
                return UP;
            default:
                std::cout << "Invalid direction!" << std::endl;
                return direction;
        }
    }

    bool forwardIsFree(int &current_x, int &current_y, direc direction) const {
        switch (direction) {
            case UP:
                 if (maze.isFree(current_x, current_y - 1))
                     return true;
                break;
            case DOWN:
                 if (maze.isFree(current_x, current_y + 1))
                     return true;
                break;
            case LEFT:
                 if (maze.isFree(current_x - 1, current_y))
                     return true;
                break;
            case RIGHT:
                 if (maze.isFree(current_x + 1, current_y))
                     return true;
                break;
            default:
                std::cout << "Invalid direction!" << std::endl;
                break;
        }
        return false;
    }

    void moveForward(int &current_x, int &current_y, direc direction) const {
        switch (direction) {
            case UP:
                current_y -= 1;  // Move up by decreasing y
                break;
            case DOWN:
                current_y += 1;  // Move down by increasing y
                break;
            case LEFT:
                current_x -= 1;  // Move left by decreasing x
                break;
            case RIGHT:
                current_x += 1;  // Move right by increasing x
                break;
            default:
                std::cout << "Invalid direction!" << std::endl;
                break;
        }
    }

    //only intersections and dead ends should be children
    void searchCorridor(direc direction, std::vector<Position>& generated){
        int current_x = x;
        int current_y = y;
        int current_d = 0;
        bool dont_save = false;

        if (!forwardIsFree(current_x, current_y, direction)) {
            dont_save = true;
            //std::cout << "Forward blocked, BLOCKED. End search completely" << std::endl;
        }

        if(dont_save == false){

            if (forwardIsFree(current_x, current_y, direction)) { //walk forward one time so one is not in a corridor anymore
                moveForward(current_x, current_y, direction);
                current_d++;

                if(maze.end().x == current_x && maze.end().y == current_y) {
                        Position tempPosit(current_x, current_y, 0);
                        std::cout << "Goal reached at (" << current_x << ", " << current_y << ")" << std::endl;
                }
            }

            while (is_corridor(current_x, current_y)) {

                if (forwardIsFree(current_x, current_y, direction)) {
                    moveForward(current_x, current_y, direction);
                    current_d++;

                    if(maze.end().x == current_x && maze.end().y == current_y) {
                            Position tempPosit(current_x, current_y, 0);
                            std::cout << "Goal reached at (" << current_x << ", " << current_y << ")" << std::endl;
                            break;
                    }
                }

                else {
                    //std::cout << "Forward blocked, checking sides..." << std::endl;

                    if (rightIsFree(current_x, current_y, direction) && leftIsFree(current_x, current_y, direction)) {
                        //std::cout << "Both sides free: INTERSECTION, end search." << std::endl;
                        break;
                    }

                    if (rightIsFree(current_x, current_y, direction)) {
                        direction = turnRight(direction);
                    }

                    else if (leftIsFree(current_x, current_y, direction)) {
                        direction = turnLeft(direction);
                    }

                    else {
                        //std::cout << "Both sides blocked: DEAD END, end search." << std::endl;
                        break;
                    }
                }
            }
            generated.push_back(Position(current_x, current_y, current_d));
        }
    }


    std::vector<Position> children() {
        std::vector<Position> generated;

//        std::cout << "\nPosition: (" << x << ", " << y << ")" << std::endl;

        searchCorridor(RIGHT, generated);
        searchCorridor(LEFT, generated);
        searchCorridor(UP, generated);
        searchCorridor(DOWN, generated);

//        std::cout << "Generated children positions: " << std::endl;
//        for (const auto& child : generated) {
//            std::cout << "(" << child.x << ", " << child.y << ", " << child.dist << ")" << std::endl;
//        }

        return generated;
    }











void print(const Point &parent) const override{

    int current_x = x;
    int current_y = y;

    std::vector<std::vector<int>> generated;
    generated.clear();

    //std::cout << "Starting print. Current point: (" << current_x << ", " << current_y << "). Parent point: (" << parent.x << ", " << parent.y << ")---------------------------------------------" << std::endl;

    if(generated.empty()){
        searchCorridorPrint(RIGHT, generated, current_x, current_y, parent.x, parent.y);
    }
    if(generated.empty()){
        searchCorridorPrint(LEFT, generated, current_x, current_y, parent.x, parent.y);
    }
    if(generated.empty()){
        searchCorridorPrint(UP, generated, current_x, current_y, parent.x, parent.y);
    }
    if(generated.empty()){
        searchCorridorPrint(DOWN, generated, current_x, current_y, parent.x, parent.y);
    }

    for(const auto& pos : generated) {
        //std::cout << "Passing through position: (" << pos[0] << ", " << pos[1] << ")" << std::endl;
        maze.passThrough(pos[0], pos[1]);
    }
}

void searchCorridorPrint(direc direction, std::vector<std::vector<int>>& generated, int current_x, int current_y, int parent_x, int parent_y) const {
    bool dont_save = false;

    if (!forwardIsFree(current_x, current_y, direction)) {
        dont_save = true;
    }

    if(dont_save == false) {
        generated.push_back({current_x, current_y});

        if (forwardIsFree(current_x, current_y, direction)) { //walk forward one time so one is not in a corridor anymore
            moveForward(current_x, current_y, direction);
            generated.push_back({current_x, current_y});
        }

        while (is_corridor(current_x, current_y)) {

            if (forwardIsFree(current_x, current_y, direction)) {
                moveForward(current_x, current_y, direction);
                generated.push_back({current_x, current_y});
            }
            else {

                if (rightIsFree(current_x, current_y, direction) && leftIsFree(current_x, current_y, direction)) {
                    break;
                }

                if (rightIsFree(current_x, current_y, direction)) {
                    direction = turnRight(direction);
                }
                else if (leftIsFree(current_x, current_y, direction)) {
                    direction = turnLeft(direction);
                }
                else {
                    break;
                }
            }
        }

        if (current_x == parent_x && current_y == parent_y) {
            generated.push_back({current_x, current_y});
        }
        else {
            generated.clear();
        }
    }
}




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

    std::cout << "End @ (" << goal.x << ", " << goal.y << ")\n";

    std::cout << "searching ..." << std::endl;
    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("cell");
    cv::waitKey(0);

}
