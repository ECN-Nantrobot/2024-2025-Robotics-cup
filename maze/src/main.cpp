#include <a_star.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <elastic_bands.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <maze.h>
#include <point.h>
#include <position.h>

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


    std::vector<Position> astar_path;
    std::vector<Position> astar_path_previous;
    ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.resetOptimization(); // Setzt die Optimierung zurück


    // Simulation parameters
    const int scale        = 20; // size up visualization for better quality
    const int display_res  = 1150;
    const float dt         = 0.05;
    const int speed_up_vis = 1; // Speed up visualization by a factor of x
    cv::Mat simulation;

    int counter_set_eb_path = 0;

    float K_P = 10;
    float K_I = 0.01;
    float K_D = 0.5;

    Robot robot(start.x, start.y, 0, 15, 7, K_P, K_I, K_D); // Initial position (x, y, theta), wheelbase 1.5, speed in cm/s, PID constants

    std ::cout << "Start Position: (" << start.x << ", " << start.y << ")" << std::endl;
    std ::cout << "Robot Position: (" << robot.getX() << ", " << robot.getY() << ")" << std::endl;


    std::vector<Obstacle> obstacles = { Obstacle(30, 100, 30, 25, Obstacle::MOVABLE, "green", 0, 8 * dt, 0), 
    Obstacle(0, 0, 25, 15, Obstacle::MOVABLE, "green", 0, 8 * dt, 8 * dt),
                                        Obstacle(299, 25, 25, 15, Obstacle::MOVABLE, "green", 0, -8 * dt, 2 * dt) };


    cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
    std::string window_name = ("Robot Simulation");
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, display_res * simulation.cols / simulation.rows, display_res * simulation.rows / simulation.rows);
    cv::moveWindow(window_name, 1, 1);


    std::chrono::duration<double> elapsedSecondsAstar;
    std::chrono::duration<double> elapsedSecondsEb;
    auto startTimeRealtime = std::chrono::steady_clock::now();


    int t = 0;
    while (true) {
        auto startTimeLoop = std::chrono::steady_clock::now();

        if (t % (int)(0.15 / dt) == 0) {
            std::cout << "Time: " << t * dt << std::endl;
        }

        // every 1s
        if (t % (int)(1 / dt) == 0) {
            start.x = robot.getX();
            start.y = robot.getY();

            double distance = sqrt(pow(start.x - goal.x, 2) + pow(start.y - goal.y, 2));
            if (distance < 6.0) {
                std::cout << "Robot is within the radius of the goal!\n" << std::endl;
                break;
            }
        }

        for (auto& obstacle : obstacles) {
            obstacle.update();
        }
        Point::maze.renderObstacles(obstacles, Point::maze.im);

        // A*: At the start and every 3s but not when elastic band is being recalculated (0s, 3s, 9s, 15s, )
        if (t == 0 || t % (int)(1 / dt) == 0 || force_recalculate == true) { //&& t % (int)(2 / dt) != 0
            auto startTimeAstar = std::chrono::steady_clock::now();

            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p    = Position(static_cast<int>(robot.getX() * Point::maze.resize_for_astar), static_cast<int>(robot.getY() * Point::maze.resize_for_astar));
            goal_p     = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
            astar_path = Astar(start_p, goal_p);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = Position(start);
                astar_path.back()  = Position(goal);
            }

            // Check the difference in the A* path
            double path_difference = 0.0;
            int limit = astar_path.size() < 25 ? astar_path.size() : astar_path.size() / 2;
            if (astar_path_previous.size() != 0) {
                for (size_t i = 1 + (astar_path.size() - limit); i < astar_path.size(); ++i) {
                    double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                    double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                    path_difference += sqrt(dx * dx + dy * dy);
                    // std ::cout << astar_path_previous[astar_path_previous.size() - i].x << " " << astar_path[astar_path.size() - i].x << std::endl;
                    //  std::cout << "dx: " << dx << " dy: " << dy << " path_difference: " << path_difference << std::endl;
                }
                path_difference /= astar_path.size();
                std ::cout << "Path difference %: " << path_difference << "  A* Path size: " << astar_path.size() << std::endl;
            }
            if (path_difference > 3.5 || astar_path_previous.size() == 0) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path); // Update the existing elastic band with the new path
                counter_set_eb_path = 0;
            }


            force_recalculate   = false;
            elastic_band.resetOptimization();

            auto endTimeAstar                                 = std::chrono::steady_clock::now();
            elapsedSecondsAstar = endTimeAstar - startTimeAstar;
            std::cout << "A* completed in: " << (elapsedSecondsAstar.count() * 1000) << " ms" << std::endl;
        }


        if (t == 0) {
            elastic_band.runFullOptimization(start, goal);
        }

        // Elastic Band
        if (!elastic_band.isOptimizationComplete()) { // t % (int)(0.5 / dt) == 0 ||
            auto startTimeEb = std::chrono::steady_clock::now();

            for (int i = 0; i < 2; i++) {
                elastic_band.optimize(start, goal);
            }

            if (elastic_band.isOptimizationComplete()) {
                std::cout << "Elastic Band optimization completed" << std::endl;
                counter_set_eb_path++;
                elastic_band.resetOptimization();
            }

            if (counter_set_eb_path >= 2) {
                elastic_band.generateSmoothedPath(0.8f, 21, 1.2f);
            }

            auto endTimeEb                                 = std::chrono::steady_clock::now();
            elapsedSecondsEb = endTimeEb - startTimeEb;
            // std::cout << "Elastic Band completed in: " << (elapsedSecondsEb.count() * 1000) << " ms" << std::endl;

            // if (action == 27) {
            //     break;
            // } else if (action == 0) {
            //     continue;
            // } else if (action == 100) {
            //     force_recalculate = true;
            // }
        }


        robot.followPath(elastic_band.getSmoothedPath(), dt);

        auto startTimeDraw = std::chrono::steady_clock::now();

        cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
        robot.draw(simulation, elastic_band.getSmoothedPath(), scale, elastic_band.getInitialPath()); // takes 1ms
        cv::imshow(window_name, simulation);

        auto endTimeDraw                                 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSecondsDraw = endTimeDraw - startTimeDraw;
        // std::cout << "Draw completed in: " << (elapsedSecondsDraw.count() * 1000) << " ms" << std::endl;

        auto endTimeLoop                                 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSecondsLoop = endTimeLoop - startTimeLoop;
        // std::cout << "Loop (without draw) completed in: " << (elapsedSecondsLoop.count() * 1000) << " ms = Astar: " << elapsedSecondsAstar.count() * 1000
        //           << " + Eb: " << elapsedSecondsEb.count() * 1000 << std::endl;

        if (elapsedSecondsLoop.count() * 1000 < dt * 1000) {
            // std::cout << "waiting for: " << static_cast<int>(std::ceil(dt * 1000 - elapsedSeconds.count() * 1000)) << " ms" << std::endl;
            cv::waitKey(static_cast<int>(std::ceil(dt * 1000 - elapsedSecondsLoop.count() * 1000)));
        } else if ((elapsedSecondsLoop.count() - elapsedSecondsDraw.count()) * 1000 > dt * 1000) {
            std ::cout << "Loop took longer than dt!!!!!!!!!!!!!!!!!!!!!!!!!: " << static_cast<int>(elapsedSecondsLoop.count() * 1000) - dt * 1000 << " ms" << std::endl;
            cv::waitKey(1);
        }

        auto endTimeRealtime                                 = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSecondsRealtime = endTimeRealtime - startTimeRealtime;
        if (t % 20 == 0) {
            std::cout << "RealTime: " << std::fixed << std::setprecision(3) << elapsedSecondsRealtime.count() << " s" << std::endl;
        }

        t++;
    }

    // const std::vector<Point>& optimizedPath = elastic_band.getPath();

    // elastic_band.generateSmoothedPath(0.8f, 22, 1.2f); // max. gap, window size, sigma

    // elastic_band.savePathToFile(filename_eb_path);

    // Point::maze.setElasticBandPath(optimizedPath);

    // std::vector<Point> astar_path_point;

    // for (const auto& position : astar_path) {
    //     astar_path_point.push_back(Point(position.x, position.y));
    // }

    // std::cout << "Search completed. Saving solution..." << std::endl;
    // Point::maze.saveSolution("solved", astar_path_point, elastic_band.getSmoothedPath(), obstacles);
    // cv::waitKey(0);

    return 0;
}