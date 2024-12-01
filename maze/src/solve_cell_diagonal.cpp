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


    // Elastic Bands optimieren den Pfad
    ecn::ElasticBand elastic_band(initial_path, Position::maze);
    elastic_band.optimize();

    // Visualisiere den optimierten Pfad
    const auto& optimizedPath = elastic_band.getPath();
    //cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    //elastic_band.drawCircles(image, cv::Scalar(0, 0, 255), 1); // Zeichne in Rot
    //elastic_band.drawPath(image, cv::Scalar(0, 0, 255), 1); // Zeichne in Rot
    //cv::imshow("Elastic Bands Path", image);

    // Convert the path to cv::Point for saving
    std::vector<cv::Point> elasticBandPathCv;
    for (const auto& pos : optimizedPath) {
        elasticBandPathCv.emplace_back(pos.x, pos.y);
    }

    // Save the solution
    std::cout << "Search completed. Saving solution..." << std::endl;
    Position::maze.saveSolution("combined", elasticBandPathCv); // No return value, so no condition here
    std::cout << "Solution saved successfully to 'combined.png'" << std::endl;

    cv::waitKey(0);

    return 0;
}