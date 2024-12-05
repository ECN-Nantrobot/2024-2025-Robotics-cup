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

// Function to find the start (blue) and end (red) points in the maze
std::pair<Point, Point> findStartAndEnd(const std::string& filename) {
    cv::Mat mazeImage = cv::imread(filename);
    if (mazeImage.empty()) {
        throw std::runtime_error("Failed to load maze image: " + filename);
    }

    Point start(-1, -1), end(-1, -1);

    // Define blue and red color ranges (in BGR format)
    cv::Scalar lowerBlue(200, 0, 0), upperBlue(255, 50, 50);
    cv::Scalar lowerRed(0, 0, 200), upperRed(50, 50, 255);

    for (int y = 0; y < mazeImage.rows; ++y) {
        for (int x = 0; x < mazeImage.cols; ++x) {
            cv::Vec3b color = mazeImage.at<cv::Vec3b>(y, x);
            // Check for blue (start point)
            if (color[0] >= lowerBlue[0] && color[0] <= upperBlue[0] &&
                color[1] >= lowerBlue[1] && color[1] <= upperBlue[1] &&
                color[2] >= lowerBlue[2] && color[2] <= upperBlue[2]) {
                start = Position(x, y);
            }
            // Check for red (end point)
            if (color[0] >= lowerRed[0] && color[0] <= upperRed[0] &&
                color[1] >= lowerRed[1] && color[1] <= upperRed[1] &&
                color[2] >= lowerRed[2] && color[2] <= upperRed[2]) {
                end = Position(x, y);
            }
        }
    }

    if (start.x == -1 || end.x == -1) {
        throw std::runtime_error(" !!! Failed to find coloured start (blue) or end points (red) in the maze.");
    }

    return {start, end};
}


void saveAstarPathToFile(const std::string& filename, const std::vector<Position>& path) {
    std::ofstream outFile(filename);
    for (const auto& pos : path) {
        outFile << pos.x << " " << pos.y << "\n";
    }
}

void saveEBPathToFile(const std::string& filename, const std::vector<FPosition>& path) {
    std::ofstream outFile(filename);
    for (const auto& pos : path) {
        outFile << pos.x << " " << pos.y << "\n";
    }
}

std::vector<Position> loadPathFromFile(const std::string& filename) {
    std::vector<Position> path;
    std::ifstream inFile(filename);
    Position pos;
    while (inFile >> pos.x >> pos.y) {
        path.push_back(pos);
    }
    return path;
}

bool fileExists(const std::string& filename) {
    return std::filesystem::exists(filename);
}

std::filesystem::file_time_type getFileModificationTime(const std::string& filename) {
    return std::filesystem::last_write_time(filename);
}





int main(int argc, char **argv)
{
    bool forceRecalculate = false;
    if (argc == 2 && std::string(argv[1]) == "new") {
            forceRecalculate = true; // Force recalculation of the A* path
    }

    // CHOOSE WHICH MAZE YOU WANT TO USE
    std::string filename_maze = Maze::mazeFile("maze_generated.png");
    //std::string filename_maze = Maze::mazeFile("maze_generated_interactive.png");

    std::string filename_astar_path = Maze::mazeFile("generated_astar_path.txt");
    std::string filename_eb_path = Maze::mazeFile("generated_eb_path.txt");

    Position::maze.load(filename_maze);

    

    // Load or calculate the A* path
    std::vector<Position> astar_path;

    if (forceRecalculate == false && fileExists(filename_astar_path) && (getFileModificationTime(filename_astar_path) >= getFileModificationTime(filename_maze))) {
        std::cout << "Loading precomputed path from file: " << filename_astar_path << std::endl;
        astar_path = loadPathFromFile(filename_astar_path);

    } else {
        Position start = Position::maze.start(), goal = Position::maze.end();
        // Check if it's an interactive maze and find start/end based on colors
        if (filename_maze.find("interactive") != std::string::npos) {
            std::cout << "Detecting start and end points based on colors..." << std::endl;
            auto [detectedStart, detectedEnd] = findStartAndEnd(filename_maze);
            start = detectedStart;
            goal = detectedEnd;
            std::cout << "Detected Start: (" << start.x << ", " << start.y << ")" << std::endl;
            std::cout << "Detected Goal: (" << goal.x << ", " << goal.y << ")" << std::endl;
        } else {
            start = Position::maze.start();
            goal = Position::maze.end();
            std::cout << "Start position: (" << start.x << ", " << start.y << ")" << std::endl;
            std::cout << "Goal position: (" << goal.x << ", " << goal.y << ")" << std::endl;
        }

        std::cout << "Starting A* search..." << std::endl;
        astar_path = ecn::Astar(start, goal);

        saveAstarPathToFile(filename_astar_path, astar_path);
        std::cout << "Path saved to file: " << filename_astar_path << std::endl;
    }

    std::vector<FPosition> fPosition_astar_path;
    for (const auto& pos : astar_path) {
        // Convert each Position to FPosition
        fPosition_astar_path.push_back(FPosition(static_cast<float>(pos.x), static_cast<float>(pos.y)));
    }
    ecn::ElasticBand elastic_band(fPosition_astar_path, Position::maze);
    elastic_band.optimize(2); //use every ... point


    const auto& optimizedPath = elastic_band.getPath();

    saveEBPathToFile(filename_eb_path, optimizedPath);
    std::cout << "Path saved to file: " << filename_eb_path << std::endl;

    
    // Convert the path to cv::Point for saving
    std::vector<cv::Point> astar_path_cv;
    for (const auto& pos : astar_path) {
        astar_path_cv.emplace_back(pos.x, pos.y);
    }
    std::vector<cv::Point> elasticband_path_cv;
    for (const auto& pos : optimizedPath) {
        elasticband_path_cv.emplace_back(pos.x, pos.y);
    }

    // Save the solution
    std::cout << "Search completed. Saving solution..." << std::endl;
    Position::maze.saveSolution("combined", astar_path_cv, elasticband_path_cv); // No return value, so no condition here
    std::cout << "Solution saved successfully to 'combined.png'" << std::endl;

    cv::waitKey(0);

    return 0;
}