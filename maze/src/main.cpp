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

    filename_maze = findLatestInteractFile();
    if (filename_maze.empty()) {
        std::cerr << "No suitable maze file found!" << std::endl;
        filename_maze = Maze::mazeFile("maze_gen_interac.png");
    } else {
        std::cout << "Using the latest maze file: " << filename_maze << std::endl;
    }


    Point::maze.load(filename_maze);

    std::string filename_astar_path = Maze::mazeFile("gen_astar_path.txt");
    std::string filename_eb_path    = Maze::mazeFile("gen_eb_path.txt");

    std::vector<Position> astar_path;
    Point start;
    Point goal;
    Position start_p;
    Position goal_p;

    // Load or calculate the A* path
    // if (force_recalculate == false && std::filesystem::exists(filename_astar_path) &&
    //     (std::filesystem::last_write_time(filename_astar_path) >= std::filesystem::last_write_time(filename_maze)))
    // {
    //     astar_path = loadPathFromFile(filename_astar_path);
    //     start      = astar_path.front();
    //     goal       = astar_path.back();
    // }
    // else
    // {
    if (filename_maze.find("interac") != std::string::npos) {
        start = Point::maze.findStart();
        goal  = Point::maze.findGoal();
    } else {
        start = Point::maze.findCornerStart();
        goal  = Point::maze.findCornerGoal();
    }

    start_p = start;
    goal_p  = goal;

    std::cout << "Searching with A* ..." << std::endl;
    astar_path = Astar(start_p, goal_p);

    saveAstarPathToFile(filename_astar_path, astar_path);
    // }


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
            astar_path = Astar(start_p, goal_p);
            saveAstarPathToFile(filename_astar_path, astar_path);
            elastic_band.updatePath(astar_path); // Update the existing elastic band with the new path
            force_recalculate = false;
        }

        int action = elastic_band.optimize(); // elastic band function

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

    std::cout << "Optimized Path:" << std::endl;
    for (const auto& point : optimizedPath) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }

    elastic_band.generateSmoothedPath(optimizedPath, 0.8f, 22, 1.2f); // max. gap, window size, sigma

    std::cout << "Smoothed Path:" << std::endl;
    const std::vector<Point>& smoothedPath = elastic_band.getSmoothedPath();
    for (const auto& point : smoothedPath) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }

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