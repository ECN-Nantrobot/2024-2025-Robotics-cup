#include <a_star.h>
#include <cstdlib>
#include <ctime>
#include <elastic_bands.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <maze.h>
#include <point.h>
#include <position.h>

using namespace std;
using namespace ecn;

int main(int argc, char** argv)
{
    srand(static_cast<unsigned int>(time(0)));

    bool force_recalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new") {
        force_recalculate = true; // Force recalculation of the A* path
    }

    std::string filename_maze;
    filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);

    std::string filename_astar_path = Maze::mazeFile("gen_astar_path.txt");
    std::string filename_eb_path    = Maze::mazeFile("gen_eb_path.txt");

    std::vector<Position> astar_path;
    Point start = Point::maze.findStart();
    Point goal  = Point::maze.findGoal();

    Point::maze.resize_for_astar = 0.5;

    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));

    std::cout << "Start: (" << start_p.x << ", " << start_p.y << ")" << std::endl;
    std::cout << "Goal: (" << start_p.x << ", " << start_p.y << ")" << std::endl;

    std::cout << "Searching with A* ..." << std::endl;
    astar_path = Astar(start_p, goal_p);

    for (auto& position : astar_path) {
        position           = position * (1 / Point::maze.resize_for_astar);
        astar_path.front() = Position(start);
        astar_path.back()  = Position(goal);
    }

    saveAstarPathToFile(filename_astar_path, astar_path);


    std::vector<Obstacle> obstacles = {
        Obstacle(0, 100, 35, 20, Obstacle::MOVABLE, "green", 0, 6, 0),
        Obstacle(0, 0, 25, 20, Obstacle::MOVABLE, "green", 0, 5, 5),
    };

    ElasticBand elastic_band(astar_path, Point::maze);


    int t = 0;
    while (true) {
        std::cout << "" << std::endl;
        std::cout << "TIME: " << t << std::endl;

        elastic_band.generateSmoothedPath(0.8f, 21, 1.2f);
        start = elastic_band.getSmoothedPath()[8];
        std::cout << "Moved Start: (" << start.x << ", " << start.y << ")" << std::endl;	

        double distance = sqrt(pow(start.x - goal.x, 2) + pow(start.y - goal.y, 2));
        if (distance < 6.0) {
            std::cout << "Start is within the radius of the goal!!!!!!!. Exiting loop.\n" << std::endl;
            break;
        }


        if (t % 5 == 0) {
            for (int j = 0; j < 1; ++j) {
                int x      = 50 + rand() % 201; // Random x between 50 and 250
                int y      = 50 + rand() % 101; // Random y between 50 and 150
                int width  = 8 + rand() % 10;   // Random width between 10 and 50
                int height = 10 + rand() % 10;  // Random height between 10 and 50
                obstacles.push_back(Obstacle(x, y, width, height, Obstacle::TEMPORARY, "green", 2));
            }
        }
        for (auto& obstacle : obstacles) {
            obstacle.update();
        }
        Point::maze.renderObstacles(obstacles, Point::maze.im);


        if (t % 4 == 0 || force_recalculate) {
            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            std::cout << "Downsized the image for A* by " << Point::maze.resize_for_astar << std::endl;
            start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
            goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
            astar_path = Astar(start_p, goal_p);
            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = Position(start);
                astar_path.back()  = Position(goal);
            }
            elastic_band.updatePath(astar_path); // Update the existing elastic band with the new path
            force_recalculate = false;
        }

        int action = elastic_band.optimize(start, goal); // elastic band function

        if (action == 27) {
            break;
        } else if (action == 0) {
            continue;
        } else if (action == 100) {
            force_recalculate = true;
        }

        t++;
    }

    const std::vector<Point>& optimizedPath = elastic_band.getPath();

    elastic_band.generateSmoothedPath(0.8f, 22, 1.2f); // max. gap, window size, sigma

    elastic_band.savePathToFile(filename_eb_path);

    Point::maze.setElasticBandPath(optimizedPath);

    std::vector<Point> astar_path_point;

    for (const auto& position : astar_path) {
        astar_path_point.push_back(Point(position.x, position.y));
    }

    std::cout << "Search completed. Saving solution..." << std::endl;
    Point::maze.saveSolution("solved", astar_path_point, elastic_band.getSmoothedPath(), obstacles);

    cv::waitKey(0);
    return 0;
}