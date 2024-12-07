#include <a_star.h>
#include <elastic_bands.h>
#include <maze.h>
#include <point.h>
#include <position.h>

#include <filesystem>
#include <fstream>
#include <iostream>

using namespace std;
using namespace ecn;


int main(int argc, char** argv)
{
    bool forceRecalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new")
    {
        forceRecalculate = true; // Force recalculation of the A* path
    }

    // CHOOSE WHICH MAZE YOU WANT TO USE
    // std::string filename_maze = Maze::mazeFile("maze_generated.png");
    std::string filename_maze = Maze::mazeFile("maze_gen_interac.png");

    Point::maze.load(filename_maze);

    std::string filename_astar_path = Maze::mazeFile("gen_astar_path.txt");
    std::string filename_eb_path    = Maze::mazeFile("gen_eb_path.txt");

    std::vector<Position> astar_path;
    Point start;
    Point goal;

    // Load or calculate the A* path
    if (forceRecalculate == false && std::filesystem::exists(filename_astar_path) &&
        (std::filesystem::last_write_time(filename_astar_path) >= std::filesystem::last_write_time(filename_maze)))
    {
        astar_path = loadPathFromFile(filename_astar_path);
        start      = astar_path.front();
        goal       = astar_path.back();
    }
    else
    {
        if (filename_maze.find("interac") != std::string::npos)
        {
            start = Point::maze.findStart();
            goal  = Point::maze.findGoal();
        }
        else
        {
            start = Point::maze.findCornerStart();
            goal  = Point::maze.findCornerGoal();
        }

        Position start_p = start;
        Position goal_p  = goal;

        std::cout << "Starting A* search..." << std::endl;
        astar_path = Astar(start_p, goal_p);

        saveAstarPathToFile(filename_astar_path, astar_path);
    }

    ecn::ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.optimize(2); // use every ... point

    const std::vector<Point>& optimizedPath = elastic_band.getPath();

    elastic_band.savePathToFile(filename_eb_path);

    Point::maze.setElasticBandPath(optimizedPath);

    std::cout << "Search completed. Saving solution..." << std::endl;
    Point::maze.saveSolution("solved");

    cv::waitKey(0);
    return 0;
}