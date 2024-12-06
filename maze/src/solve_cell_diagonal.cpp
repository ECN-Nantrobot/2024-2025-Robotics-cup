#include <a_star.h>
#include <maze.h>
#include <elastic_bands.h>
#include <point.h>
#include <position.h>

#include <filesystem>
#include <iostream>
#include <fstream>

using namespace std;
using namespace ecn;




int main(int argc, char **argv)
{
    // ecn::Maze maze_instance;
    // ecn::Point::maze = &maze_instance; // Assign Maze instance to static pointer
    
    bool forceRecalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new") {
            forceRecalculate = true; // Force recalculation of the A* path
    }

    // CHOOSE WHICH MAZE YOU WANT TO USE
    //std::string filename_maze = Maze::mazeFile("maze_generated.png");
    std::string filename_maze = Maze::mazeFile("maze_generated_interactive.png");
    Point::maze.load(filename_maze);

    std::string filename_astar_path = Maze::mazeFile("generated_astar_path.txt");
    std::string filename_eb_path = Maze::mazeFile("generated_eb_path.txt");

    // Load or calculate the A* path
    std::vector<Position> astar_path;
    Point start;
    Point goal;

    if (forceRecalculate == false && std::filesystem::exists(filename_astar_path) && (std::filesystem::last_write_time(filename_astar_path) >= std::filesystem::last_write_time(filename_maze)))
    {
        std::cout << "Loading precomputed path from file: " << filename_astar_path << std::endl;
        astar_path = loadPathFromFile(filename_astar_path);
        start = astar_path.front();
        goal = astar_path.back();
        }
    else
    {
        // Position start = Position::maze.start(), goal = Position::maze.end();
        // Check if it's an interactive maze and find start/end based on colors
        if (filename_maze.find("interactive") != std::string::npos) {
            std::cout << "Detecting start and end points based on colors..." << std::endl;
            start = Point::maze.findStart();
            goal = Point::maze.findGoal();
        } else {
            start = Point::maze.findCornerStart();
            goal = Point::maze.findCornerGoal();
        }

        Position start_p = start;
        Position goal_p = goal;

        std::cout << "Starting A* search..." << std::endl;
        astar_path = Astar(start_p, goal_p);

        saveAstarPathToFile(filename_astar_path, astar_path);
        std::cout << "Path saved to file: " << filename_astar_path << std::endl;
    }

    ecn::ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.optimize(2); //use every ... point

    const std::vector<Point>& optimizedPath = elastic_band.getPath();

    elastic_band.savePathToFile(filename_eb_path);

    Point::maze.setElasticBandPath(optimizedPath);

    // Convert the path to cv::Point for saving
    // std::vector<cv::Point> astar_path_cv;
    // for (const auto& pos : astar_path) {
    //     astar_path_cv.emplace_back(pos.x, pos.y);
    // }

    // std::vector<cv::Point> elasticband_path_cv;
    // for (const auto& pos : optimizedPath) {
    //     elasticband_path_cv.emplace_back(pos.x, pos.y);
    // }

    std::cout << "Search completed. Saving solution..." << std::endl;
    Point::maze.saveSolution("combined");
    std::cout << "Solution saved successfully to 'combined.png'" << std::endl;

    cv::waitKey(0);

    return 0;
}