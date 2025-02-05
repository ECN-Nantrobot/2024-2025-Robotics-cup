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


enum RobotState { INIT, PATH_PLANNING, NAVIGATION, TURN_TO_GOAL, TURN_TO_PATH, GOAL_REACHED };


int main()
{
    RobotState state = INIT;

    std::string filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png"); // CHOOSE WHICH MAZE YOU WANT TO USE
    Point::maze.load(filename_maze);
    Point::maze.computeDistanceTransform(); // Precompute distance transform

    // Point start      = Point::maze.findStart();
    // Point goal       = Point::maze.findGoal();
    Point start;
    Point goal;
    Position start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    Position goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));

    std::vector<Point> goals = { Point(250, 180), Point(50, 15), Point(80, 150), Point(225, 80) }; // Example goal points
    int current_goal_index   = 1;                                                                  // Keep track of which goal the robot is targeting
    start                    = goals[0];
    std::vector<double> target_thetas(goals.size(), 0); // Initialize target thetas
    target_thetas = { -90, 0, 90, 0 };

    int counter_set_eb_path = 0;

    Robot robot(Point::maze, start.x, start.y, 0, 15, 7, 10, 0.01, 0.5); // Maze, initial position (x, y, theta), wheelbase, speed in cm/s, P, I, D
    robot.setIsStarting(true);                                           // Enable gradual start
    robot.setPose(goals[0].x, goals[0].y, target_thetas[0]);
    robot.setTargetTheta(target_thetas[0]);


    std::vector<Position> astar_path;
    std::vector<Position> astar_path_previous;
    ElasticBand elastic_band(astar_path, Point::maze);
    elastic_band.resetOptimization(); // Setzt die Optimierung zurück

    const float dt = 0.05;
    int t          = 0;

    float distance_to_goal                = robot.distanceToGoal(goal);
    int set_eb_path_counter_limit_default = 1;
    int set_eb_path_counter_limit         = set_eb_path_counter_limit_default;
    int eb_comp_inarow_default            = 1;
    int eb_comp_inarow                    = eb_comp_inarow_default;

    std::vector<Obstacle> obstacles = { Obstacle(150, 100, 35, 25, Obstacle::MOVABLE, "lightgray", 0, 5 * dt, 0),
                                        Obstacle(0, 20, 30, 15, Obstacle::MOVABLE, "lightgray", 0, 3 * dt, 1 * dt),
                                        Obstacle(299, 25, 30, 15, Obstacle::MOVABLE, "lightgray", 0, -3 * dt, 2 * dt),
                                        Obstacle(350, 80, 65, 10, Obstacle::MOVABLE, "lightgray", 0, -4 * dt, 1 * dt) };


    const int scale       = 20; // size up visualization for better quality
    const int display_res = 1300;
    cv::Mat simulation;
    cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
    cv::cvtColor(simulation, simulation, cv::COLOR_BGR2BGRA); // to support transparency
    std::string window_name = ("Robot Simulation");
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, display_res * simulation.cols / simulation.rows, display_res * simulation.rows / simulation.rows);
    cv::moveWindow(window_name, 1, 1);

    std::chrono::time_point<std::chrono::steady_clock> astarStartTime, astarEndTime, elasticStartTime, elasticEndTime, realStartTime, realEndTime, loopStartTime, loopEndTime;
    std::chrono::duration<double> astarDuration, elasticDuration, realDuration, loopDuration;
    float path_difference           = 0.0;
    float path_difference_front     = 0.0;
    float path_difference_end       = 0.0;
    int path_difference_check_limit = 0;

    realStartTime = std::chrono::steady_clock::now();

    while (true) {
        loopStartTime = std::chrono::steady_clock::now();


        // Updating the Map
        for (auto& obstacle : obstacles) {
            obstacle.update();
        }
        Point::maze.renderObstacles(obstacles, Point::maze.im);


        switch (state) {
        case INIT:
            std::cout << "State: INIT, goal: (" << goals[current_goal_index].x << ", " << goals[current_goal_index].y << ")" << std::endl;

            robot.setIsStarting(true); // Enable gradual start

            goal = goals[current_goal_index];

            Point::maze.computeDistanceTransform();
            cv::resize(Point::maze.im, Point::maze.im_lowres, cv::Size(), Point::maze.resize_for_astar, Point::maze.resize_for_astar, cv::INTER_AREA);
            start_p    = Position(static_cast<int>(robot.getX() * Point::maze.resize_for_astar), static_cast<int>(robot.getY() * Point::maze.resize_for_astar));
            goal_p     = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));
            astar_path = Astar(start_p, goal_p);

            for (auto& position : astar_path) {
                position           = position * (1 / Point::maze.resize_for_astar);
                astar_path.front() = Position(start);
                astar_path.back()  = Position(goal);
            }

            elastic_band.updatePath(astar_path);
            std::cout << "    -> A* Path Set" << std::endl;

            elastic_band.runFullOptimization(start, goal);
            std::cout << "Elastic Band Path completed full optimization, -> Path Set" << std::endl;

            state = TURN_TO_PATH;
            [[fallthrough]];


        case TURN_TO_PATH:
            std::cout << "State: TURN_TO_PATH" << std::endl;

            if (robot.turnToPathOrientation(dt, elastic_band.getSmoothedPath())) {
                std::cout << "Robot is aligned to path orientation!" << std::endl;
                robot.setIsStarting(true);
                state = NAVIGATION;
            }

            break;


        case PATH_PLANNING:
            std::cout << "State: PLANNING" << std::endl;

            astarStartTime = std::chrono::steady_clock::now();

            // Recalculate A* path
            Point::maze.computeDistanceTransform();
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
            path_difference_front = 0.0;
            path_difference_end   = 0.0;
            path_difference = 0.0;
            // path_difference_check_limit = astar_path.size() < 25 ? astar_path.size() : astar_path.size() / 2;
            path_difference_check_limit = astar_path.size()/2;
            if (astar_path_previous.size() != 0) {
                for (size_t i = 1 + path_difference_check_limit; i < astar_path.size(); i += 2) {
                    double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                    double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                    path_difference_front += sqrt(dx * dx + dy * dy);
                }
                path_difference_front /= astar_path.size();
                std ::cout << "A* path diff front: %: " << path_difference_front;

                for (size_t i = 1; i < astar_path.size() - path_difference_check_limit; i += 2) {
                    double dx = astar_path_previous[astar_path_previous.size() - i].x - astar_path[astar_path.size() - i].x;
                    double dy = astar_path_previous[astar_path_previous.size() - i].y - astar_path[astar_path.size() - i].y;
                    path_difference_end += sqrt(dx * dx + dy * dy);
                }
                path_difference_end /= astar_path.size();
                std ::cout << " A* path diff end: %: " << path_difference_end;
            }
            else {
                std::cout << "astar_path_previous.size() = " << astar_path_previous.size() << std::endl;
            }
            path_difference = path_difference_front + path_difference_end;
            if (path_difference_front > 3) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                std::cout << "    -> A* Path Front, set eb" << std::endl;
                elastic_band.resetOptimization();
                counter_set_eb_path       = 0;
                set_eb_path_counter_limit = 1;
                eb_comp_inarow            = 1;
            } else if (path_difference_end > 4) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                std::cout << "    -> A* Path Back, set eb" << std::endl;
                set_eb_path_counter_limit = 2;
                counter_set_eb_path       = 0;
                elastic_band.resetOptimization();
            } else if (path_difference >= 3.1) {
                astar_path_previous = astar_path;
                elastic_band.updatePath(astar_path);
                std::cout << "    -> A* Path FULL, set eb" << std::endl;
                elastic_band.resetOptimization();
                counter_set_eb_path       = 0;
                set_eb_path_counter_limit = 1;
                eb_comp_inarow            = 1;
                elastic_band.setMaxInterations(6);
                } else if (path_difference > 2.6 && path_difference < 3 || astar_path_previous.size() == 0) {
                    astar_path_previous = astar_path;
                    elastic_band.updatePath(astar_path);
                    std::cout << "    -> A* Path MEDIUM, set eb" << std::endl;
                    set_eb_path_counter_limit = 3;
                    counter_set_eb_path = 0;
                    elastic_band.resetOptimization();
            } else {
                std ::cout << "" << std::endl;
            }


            astarEndTime  = std::chrono::steady_clock::now();
            astarDuration = astarEndTime - astarStartTime;
            // std::cout << "A* completed in: " << std::fixed << std::setprecision(3) << astarDuration.count() * 1000 << " ms" << std::endl;

            state = NAVIGATION;
            [[fallthrough]];


        case NAVIGATION:
            std::cout << "State: NAVIGATION, goal: (" << goal.x << ", " << goal.y << ")" << std::endl;

            elasticStartTime = std::chrono::steady_clock::now();


            // for (int i = 0; i < eb_comp_inarow; i++) {
            //     Point::maze.computeDistanceTransform();
            elastic_band.optimize(start, goal);

            if (elastic_band.errorCheck() == true) {
                std::cout << "EB Optimization error -> PATH PLANNING." << std::endl;
                elastic_band.resetOptimization();
                state = PATH_PLANNING;
                break;
            }

            if (elastic_band.isOptimizationComplete()) {
                std::cout << "Elastic Band optimization completed";
                counter_set_eb_path++;
                elastic_band.resetOptimization();

                if (counter_set_eb_path > set_eb_path_counter_limit) {
                    // std ::cout << "counter_set_eb_path > set_eb_path_counter_limit: " << counter_set_eb_path << " > " << set_eb_path_counter_limit << std::endl;
                    elastic_band.generateSmoothedPath(0.8f, 21, 1.2f); // 0.08 ms
                    elastic_band.savePathToFile("newtestpath"); ///////////////////////////////////////
                    std::cout << " -> Smoothed Path Set" << std::endl;  
                    std::cout << "  -> Path Set" << std::endl;

                    // counter++;
                    // if(counter >= 2){
                    set_eb_path_counter_limit = set_eb_path_counter_limit_default;
                    eb_comp_inarow            = eb_comp_inarow_default;
                    // }

                } else {
                    std::cout << "" << std::endl;
                }
            }

            elasticEndTime  = std::chrono::steady_clock::now();
            elasticDuration = elasticEndTime - elasticStartTime;
            // std::cout << "Elastic Band optimization completed in: " << std::fixed << std::setprecision(3) << elasticDuration.count() * 1000 << " ms" << std::endl;


            robot.followPath(elastic_band.getSmoothedPath(), Point::maze, dt);
            
            start            = Point(robot.getX(), robot.getY());
            
            distance_to_goal = robot.distanceToGoal(goal);
            if (distance_to_goal < 5.0) {
                std::cout << "Distance to goal: " << distance_to_goal << std::endl;

                if (distance_to_goal < 0.5) {
                    std::cout << "Robot has reached the goal -> TURN_TO_GOAL!" << std::endl;
                    state = TURN_TO_GOAL;
                }

            } else if (t % static_cast<int>(1 / dt) == 0) {
                state = PATH_PLANNING;
            } else {
                state = NAVIGATION;
            }

            break;


        case TURN_TO_GOAL:
            std::cout << "State: TURN_TO_GOAL" << std::endl;

            if (robot.turnToGoalOrientation(dt)) {
                std::cout << "Robot is aligned to target orientation!" << std::endl;
                state = GOAL_REACHED;
            }

            break;


        case GOAL_REACHED:
            std::cout << "State: GOAL_REACHED: (" << goal.x << ", " << goal.y << ")" << std::endl;

            if (current_goal_index + 1 < goals.size()) {
                current_goal_index++;
                robot.setTargetTheta(target_thetas[current_goal_index]);
                state = INIT;
            } else {
                std::cout << "All goals have been reached. Mission complete!" << std::endl;
                return 0; // End simulation
            }
            break;
        }


        cv::resize(Point::maze.getIm(), simulation, cv::Size(), scale, scale, cv::INTER_NEAREST);
        cv::cvtColor(simulation, simulation, cv::COLOR_BGR2BGRA);                                                                    // to support transparency
        robot.draw(simulation, elastic_band.getSmoothedPath(), scale, elastic_band.getInitialPath(), goals, elastic_band.getPath(), astar_path); // takes 1ms
        cv::imshow(window_name, simulation);
        if (cv::waitKey(1) >= 0) { // play/pause with aney key
            cv::waitKey(0);
        }

        loopEndTime  = std::chrono::steady_clock::now();
        loopDuration = loopEndTime - loopStartTime;
        if (loopDuration.count() * 1000 > dt * 1000) {
            // std::cout << "WARNING: Loop took longer than dt!!!!!!!!!! Time taken: " << loopDuration.count() * 1000 << " ms, Expected: " << dt * 1000 << " ms" << std::endl;
            cv::waitKey(1);

        } else {
            // std::cout << "Loop running in real-time. Time taken: " << loopDuration.count() * 1000 << " ms" << std::endl;
            cv::waitKey(static_cast<int>(std::ceil(dt * 1000 - loopDuration.count() * 1000)));
        }

        t++;
        // std::cout << "Simulation time: " << std::fixed << std::setprecision(3) << t * dt << " s" << std::endl;
        realEndTime  = std::chrono::steady_clock::now();
        realDuration = realEndTime - realStartTime;
        // std::cout << "Real Time:       " << std::fixed << std::setprecision(3) << realDuration.count() << " s" << std::endl;
    }

    return 0;
}
