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
#include <chrono>

#include "point.h"
#include "robot.h"

using namespace std;
using namespace ecn;

int main()
{
    bool force_recalculate = false;

    std::string filename_maze;
    filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);

    std::string filename_astar_path = Maze::mazeFile("gen_astar_path.txt");
    std::string filename_eb_path    = Maze::mazeFile("gen_eb_path.txt");

    Point start      = Point::maze.findStart();
    Point goal       = Point::maze.findGoal();
    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));


    std::vector<Obstacle> obstacles = {
        Obstacle(0, 100, 35, 20, Obstacle::MOVABLE, "green", 0, 1, 0),
        Obstacle(0, 0, 25, 20, Obstacle::MOVABLE, "green", 0, 1, 1),
    };

    std::vector<Position> astar_path;
    ElasticBand elastic_band(astar_path, Point::maze);

    // Simulation parameters
    const int scale        = 20; // size up visualization for better quality
    const int display_res  = 1200;
    const float dt         = 0.02;
    const int speed_up_vis = 1; // Speed up visualization by a factor of x
    cv::Mat simulation;

    float K_P = 10;
    float K_I = 0.01;
    float K_D = 0.5;

    Robot robot(start.x, start.y, 0, 15, 8, K_P, K_I, K_D); // Initial position (x, y, theta), wheelbase 1.5, speed in cm/s, PID constants

    std ::cout << "Start Position: (" << start.x << ", " << start.y << ")" << std::endl;
    std ::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;

    int t = 0;
    while (true) {
        auto startTimeLoop = std::chrono::steady_clock::now();

        if (t % (int)(0.1 / dt) == 0) {
            std::cout << "Time: " << t * dt << std::endl;
        }

        // every 1s
        if (t % (int)(1 / dt) == 0) { 
            start.x = robot.getX();
            start.y = robot.getY();

            double distance = sqrt(pow(start.x - goal.x, 2) + pow(start.y - goal.y, 2));
            if (distance < 6.0) {
                std::cout << "Robot is within the radius of the goal!.\n" << std::endl;
                break;
            }

            for (auto& obstacle : obstacles) {
                obstacle.update();
            }
            Point::maze.renderObstacles(obstacles, Point::maze.im);
        }

        // A*: At the start and every 3s but not when elastic band is being recalculated (0s, 3s, 9s, 15s, )
        if (t == 0 || t % (int)(3 / dt) == 0 && t % (int)(2 / dt) != 0 || force_recalculate) {
            auto startTime = std::chrono::steady_clock::now();

            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p    = Position(static_cast<int>(robot.getX() * Point::maze.resize_for_astar), static_cast<int>(robot.getY() * Point::maze.resize_for_astar));
            goal_p     = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
            astar_path = Astar(start_p, goal_p);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = Position(start);
                astar_path.back()  = Position(goal);
            }
            force_recalculate                            = false;

            auto endTime                                 = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsedSeconds = endTime - startTime;
            std::cout << "A* Pathfinding completed in: " << static_cast<int>(elapsedSeconds.count() * 1000) << " ms" << std::endl;          
        }


        // Elastic Band: every 2s
        if (t % (int)(2 / dt) == 0) {
            auto startTime = std::chrono::steady_clock::now();

            elastic_band.updatePath(astar_path); // Update the existing elastic band with the new path
            int action = elastic_band.optimize(start, goal);
            elastic_band.generateSmoothedPath(0.8f, 21, 1.2f);

            auto endTime                                 = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsedSeconds = endTime - startTime;
            std::cout << "Elastic Band completed in: " << static_cast<int>(elapsedSeconds.count() * 1000) << " ms" << std::endl;

            if (action == 27) {
                break;
            } else if (action == 0) {
                continue;
            } else if (action == 100) {
                force_recalculate = true;
            }
        }


        robot.followPath(elastic_band.getSmoothedPath(), dt);



        cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
        robot.draw(simulation, elastic_band.getSmoothedPath(), scale);

        std::string window_name = ("Robot Simulation");
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, display_res * simulation.cols / simulation.rows, display_res * simulation.rows / simulation.rows);
        cv::moveWindow(window_name, 50, 50);
        cv::imshow(window_name, simulation);
        cv::waitKey(dt * 1000 / speed_up_vis - 10); //visualizatoin takes ~10ms


        auto endTime                                 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = endTime - startTimeLoop;
        // std::cout << "Loop completed in: " << static_cast<int>(elapsedSeconds.count() * 1000) << " ms" << std::endl;

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