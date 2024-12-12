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


std::string findLatestInteractFile()
{
    namespace fs = std::filesystem;

    std::string mazeDirectory = MAZES; // Automatically use the MAZES directory
    std::string latestFile;
    std::time_t latestTime = 0;

    try {
        for (const auto& entry : fs::directory_iterator(mazeDirectory)) {
            const auto& path           = entry.path();
            const std::string filename = path.filename().string();

            // Check if the file contains "interact" and does not contain "solved"
            if (filename.find("interact") != std::string::npos && filename.find("solved") == std::string::npos) {
                // Get the last write time
                auto fileTime = fs::last_write_time(path);

                // Convert file_time_type to system_clock::time_point
                auto sctpTime = std::chrono::time_point_cast<std::chrono::system_clock::duration>(fileTime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());

                // Convert to time_t
                std::time_t time = std::chrono::system_clock::to_time_t(sctpTime);

                // Update the latest file if this one is newer
                if (time > latestTime) {
                    latestTime = time;
                    latestFile = Maze::mazeFile(filename); // Use Maze::mazeFile to construct the full path
                }
            }
        }

        if (!latestFile.empty()) {
            std::cout << "Found file: " << latestFile << std::endl;
            std::cout << "Last write time: " << std::asctime(std::localtime(&latestTime)) << std::endl;
        } else {
            std::cout << "No matching file found in the directory." << std::endl;
        }

    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }

    return latestFile;
}


// // Globale Variablen für Mausinteraktionen
// std::vector<Obstacle> userDefinedObstacles;
// Maze* interactiveMaze = nullptr;

// // Callback-Funktion für Mausinteraktionen
// void onMouse(int event, int x, int y, int flags, void* userdata)
// {
//     const int scale = 5; // Skalierungsfaktor der Anzeige
//     if (event == cv::EVENT_LBUTTONDOWN) {
//         std::cout << "Mausklick erkannt bei Pixel (" << x << ", " << y << ")" << std::endl;

//         int mazeX = x / scale;
//         int mazeY = y / scale;

//         // Hindernisgröße und Parameter festlegen
//         int width  = 20; // Beispielbreite
//         int height = 20; // Beispielhöhe

//         // Neues Hindernis hinzufügen
//         userDefinedObstacles.emplace_back(mazeX, mazeY, width, height, Obstacle::TEMPORARY, "green", 10);

//         // Hindernis zur Karte hinzufügen
//         if (interactiveMaze) {
//             interactiveMaze->updateObstacles(userDefinedObstacles);
//             std::cout << "Hindernis hinzugefügt bei (" << mazeX << ", " << mazeY << ") mit Größe [" << width << "x" << height << "]" << std::endl;
//         } else {
//             std::cerr << "Maze-Zeiger ist nicht initialisiert!" << std::endl;
//         }
//     }
// }


int main(int argc, char** argv)
{
    srand(static_cast<unsigned int>(time(0)));

    bool force_recalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new") {
        force_recalculate = true; // Force recalculation of the A* path
    }

    std::string filename_maze;
    // CHOOSE WHICH MAZE YOU WANT TO USE
    // filename_maze = Maze::mazeFile("maze_generated.png");
    // filename_maze = Maze::mazeFile("maze_gen_interac.png");

    // filename_maze = findLatestInteractFile();
    // if (filename_maze.empty()) {
    // std::cerr << "No suitable maze file found!" << std::endl;
    filename_maze = Maze::mazeFile("Eurobot_map_real_bw_10_p_interact.png");
    // } else {
    // std::cout << "Using the latest maze file: " << filename_maze << std::endl;
    // }

    std::string filename_maze_lowres = Maze::mazeFile("Eurobot_map_real_w_20_p_interact.png");

    Point::maze.load(filename_maze);
    // Point::maze.loadLowRes(filename_maze_lowres);


    std::string filename_astar_path = Maze::mazeFile("gen_astar_path.txt");
    std::string filename_eb_path    = Maze::mazeFile("gen_eb_path.txt");

    std::vector<Position> astar_path;
    Point start;
    Point goal;
    Position start_p;
    Position goal_p;

    Point::maze.resize_for_astar = 0.5;


    start = Point::maze.findStart();
    goal  = Point::maze.findGoal();

    start_p = Position(static_cast<int>(start.x * Point::maze.resize_for_astar), static_cast<int>(start.y * Point::maze.resize_for_astar));
    goal_p  = Position(static_cast<int>(goal.x * Point::maze.resize_for_astar), static_cast<int>(goal.y * Point::maze.resize_for_astar));

    std::cout << "Start: (" << start_p.x << ", " << start_p.y << ")" << std::endl;
    std::cout << "Goal: (" << start_p.x << ", " << start_p.y << ")" << std::endl;

    std::cout << "Searching with A* ..." << std::endl;
    astar_path = Astar(start_p, goal_p);

    std ::cout << "A* Path size: " << astar_path.size() << std::endl;


    for (auto& position : astar_path) {
        position           = position * (1 / Point::maze.resize_for_astar);
        astar_path.front() = Position(start);
        astar_path.back()  = Position(goal);
    }


    std ::cout << "A* Path size: " << astar_path.size() << std::endl;

    saveAstarPathToFile(filename_astar_path, astar_path);


    // const int scale = 5; // Skalierungsfaktor
    // cv::Mat visualization;
    // cv::resize(Point::maze.getIm(), visualization, cv::Size(), scale, scale, cv::INTER_NEAREST);
    // cv::namedWindow("Elastic Band Optimization", cv::WINDOW_NORMAL);
    // cv::resizeWindow("Elastic Band Optimization", Point::maze.width() * scale, Point::maze.height() * scale);
    // cv::imshow("Elastic Band Optimization", visualization);
    // cv::setMouseCallback("Elastic Band Optimization", onMouse);
    // interactiveMaze = &Point::maze; // Zeiger auf das aktuelle Maze setzen


    // Create a list of obstacles
    std::vector<Obstacle> obstacles = {
        Obstacle(0, 100, 25, 10, Obstacle::MOVABLE, "green", 0, 4, 0),
        Obstacle(0, 0, 20, 15, Obstacle::MOVABLE, "green", 0, 5, 5),
    };

    ElasticBand elastic_band(astar_path, Point::maze);


    int t = 0;
    while (true) {
        std::cout << "" << std::endl;
        std::cout << "TIME: " << t << std::endl;

        // start.x -= 2;
        // start.y -= 4;

        if (t % 5 == 0) {
            for (int j = 0; j < 1; ++j) {
                // int x      = rand() % Point::maze.width();
                // int y      = rand() % Point::maze.height();
                int x      = 50 + rand() % 201; // Random x between 50 and 250
                int y      = 50 + rand() % 101; // Random y between 50 and 150
                int width  = 8 + rand() % 10;   // Random width between 10 and 50
                int height = 10 + rand() % 10;  // Random height between 10 and 50
                obstacles.push_back(Obstacle(x, y, width, height, Obstacle::TEMPORARY, "green", 2));
            }
        }

        // for (auto& obstacle : userDefinedObstacles) {
        //     obstacles.push_back(obstacle);
        // }

        for (auto& obstacle : obstacles) {
            obstacle.update();
        }
        Point::maze.renderObstacles(obstacles, Point::maze.im);


        if (t % 6 == 0 || force_recalculate) {
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
            saveAstarPathToFile(filename_astar_path, astar_path);
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

    elastic_band.generateSmoothedPath(optimizedPath, 0.8f, 22, 1.2f); // max. gap, window size, sigma

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