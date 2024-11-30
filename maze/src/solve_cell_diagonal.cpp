#include <a_star.h>
#include <maze.h>
#include <elastic_bands.h>
#include <point.h>
#include <position.h>

using namespace std;
using namespace ecn;

int main(int argc, char **argv)
{
    std::string filename = Maze::mazeFile("maze_generated.png");
    if (argc == 2)
        filename = std::string(argv[1]);
    std::cout << "Loading maze from file: " << filename << std::endl;

    // Let Point know about this maze
    Position::maze.load(filename);

    Position start = Position::maze.start(), goal = Position::maze.end();
    std::cout << "Start position: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal position: (" << goal.x << ", " << goal.y << ")" << std::endl;

    std::cout << "Starting A* search..." << std::endl;
    vector<Position> initial_path = ecn::Astar(start, goal);


    // // Create and optimize the elastic band
    // ElasticBand elastic_band(initial_path, Position::maze);
    // elastic_band.optimize();

    // const vector<Position>& optimized_path = elastic_band.getPath();

    // cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    // elastic_band.drawCircles(image, cv::Scalar(0, 0, 255)); // Draw in red color
    // cv::imshow("Elastic Bands Path", image);


    // Save the solution
    std::cout << "Search completed. Saving solution..." << std::endl;
    Position::maze.saveSolution("cell"); // No return value, so no condition here
    std::cout << "Solution saved successfully to 'cell.png'" << std::endl;

    cv::waitKey(0);

    return 0;
}