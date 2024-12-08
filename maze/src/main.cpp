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

    bool forceRecalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new") {
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
        (std::filesystem::last_write_time(filename_astar_path) >= std::filesystem::last_write_time(filename_maze))) {
        astar_path = loadPathFromFile(filename_astar_path);
        start      = astar_path.front();
        goal       = astar_path.back();
    } else {
        if (filename_maze.find("interac") != std::string::npos) {
            start = Point::maze.findStart();
            goal  = Point::maze.findGoal();
        } else {
            start = Point::maze.findCornerStart();
            goal  = Point::maze.findCornerGoal();
        }

        Position start_p = start;
        Position goal_p  = goal;

        std::cout << "Searching with A* ..." << std::endl;
        astar_path = Astar(start_p, goal_p);

        saveAstarPathToFile(filename_astar_path, astar_path);
    }


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
        Obstacle(65, 70, 30, 20, Obstacle::MOVABLE, "green"), 
        // Obstacle(150, 55, 40, 30, Obstacle::FIXED, "green"),
    };

    ecn::ElasticBand elastic_band(astar_path, Point::maze);

    int t = 0;
    while (true) {
        std::cout << "" << std::endl;
        std::cout << "TIME: " << t << std::endl;

        for (int j = 0; j < 6; ++j) {
            int x      = rand() % Point::maze.width();
            int y      = rand() % Point::maze.height();
            int width  = 8 + rand() % 10; // Random width between 10 and 50
            int height = 10 + rand() % 10; // Random height between 10 and 50
            obstacles.push_back(Obstacle(x, y, width, height, Obstacle::TEMPORARY, "green", 2));
        }
        if(t == 1){
            obstacles.push_back(Obstacle(180, 100, 70, 60, Obstacle::TEMPORARY, "green", 2));
        }

        for (int j = 0; j < 1; ++j) {
            int x      = rand() % Point::maze.width();
            int y      = rand() % Point::maze.height();
            int width  = 10 + rand() % 50; // Random width between 10 and 50
            int height = 10 + rand() % 50; // Random height between 10 and 50
            obstacles.push_back(Obstacle(x, y, width, height, Obstacle::TEMPORARY, "green", 2));
        }


        // for (auto& obstacle : userDefinedObstacles) {
        //     obstacles.push_back(obstacle);
        // }

        for (auto& obstacle : obstacles) {
            obstacle.update();
        }
        Point::maze.updateObstacles(obstacles);

        elastic_band.optimize();


        if (obstacles[0].getType() == Obstacle::MOVABLE && obstacles[0].isActive()) {
            obstacles[0].moveTo(obstacles[0].getX() + 20, obstacles[0].getY());
        }

        int key = cv::waitKey(1);
        if (key == 27) { // Press 'ESC' to exit
            break;
        }
        t++;
    }

    const std::vector<Point>& optimizedPath = elastic_band.getPath();

    elastic_band.savePathToFile(filename_eb_path);

    Point::maze.setElasticBandPath(optimizedPath);

    std::vector<Point> astar_path_point;

    for (const auto& position : astar_path) {
        astar_path_point.push_back(Point(position.x, position.y));
    }

    std::cout << "Search completed. Saving solution..." << std::endl;
    Point::maze.saveSolution("solved", astar_path_point);

    cv::waitKey(0);
    return 0;
}