#include "maze.h"
#include "point.h"
#include <cmath>

namespace ecn
{

std::string Maze::mazeFile(const std::string& file) { return std::string(MAZES) + "/" + file; }

Maze::Maze() {}

void Maze::load(std::string _filename)
{
    std::cout << "Loading " << _filename << " ... ";
    filename = _filename;

    im = cv::imread(filename, cv::IMREAD_UNCHANGED);

    if (im.empty()) {
        throw std::runtime_error("Failed to load the image at: " + filename);
    }

    if (im.channels() == 4) {
        cv::cvtColor(im, im, cv::COLOR_RGBA2BGR);
        std::cout << "Converted RGBA image to BGR format ... ";
    } else if (im.channels() == 1) {
        cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
        std::cout << "Converted grayscale image to BGR format ... ";
    }


    // Add a 5-pixel border around obstacles
    const int border_size = 5;
    cv::Mat obstacle_mask = cv::Mat::zeros(im.size(), CV_8UC1);

    // Create a mask for all pixels below the threshold
    for (int y = 0; y < im.rows; ++y) {
        for (int x = 0; x < im.cols; ++x) {
            cv::Vec3b pixel_color = im.at<cv::Vec3b>(y, x);
            if (pixel_color[0] == pixel_color[1] && pixel_color[1] == pixel_color[2]) // if the color is gray and darker than 120
            {
                if (pixel_color[0] < permanent_obstacle_treshold)
                    obstacle_mask.at<uchar>(y, x) = 255; // Mark obstacle in the mask
            }
        }
    }

    // Step 2: Dilate the obstacle mask to include the border
    cv::Mat dilated_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * border_size + 1, 2 * border_size + 1));
    cv::dilate(obstacle_mask, dilated_mask, kernel);

    // Step 3: Subtract the original obstacle mask from the dilated mask to isolate the border
    cv::Mat border_mask;
    cv::subtract(dilated_mask, obstacle_mask, border_mask);

    // Step 4: Apply the border mask to the image
    for (int y = 0; y < im.rows; ++y) {
        for (int x = 0; x < im.cols; ++x) {
            if (border_mask.at<uchar>(y, x) == 255) {
                im.at<cv::Vec3b>(y, x) = cv::Vec3b(40, 40, 40);
            }
        }
    }
    std::cout << "Added border of " << border_size << " to permanent(balck) obstacles ... ";

    original_im = im.clone();
    out         = im.clone();

    cv::resize(im, im_lowres, cv::Size(), resize_for_astar, resize_for_astar, cv::INTER_AREA);
    std::cout << "Downsized the image for A* by "
              << resize_for_astar << " ... ";

    // Save the low-resolution image
    // std::string lowres_filename = filename.substr(0, filename.find_last_of('.')) + "_lowres.png";
    // if (!cv::imwrite(lowres_filename, im_lowres)) {
    //     std::cerr << "Error: Failed to save low-resolution image to " << lowres_filename << std::endl;
    // } else {
    //     std::cout << "Saved low-resolution image to " << lowres_filename << std::endl;
    // }

    std::cout << " ok"
              << std::endl;
}

void Maze::loadLowRes(std::string _filename)
{
    std::cout << "Loading LOWRES " << _filename << " ... ";
    filename = _filename;

    im_lowres = cv::imread(filename, cv::IMREAD_UNCHANGED);

    if (im_lowres.empty()) {
        throw std::runtime_error("Failed to load the image at: " + filename);
    }

    if (im_lowres.channels() == 4) {
        cv::cvtColor(im_lowres, im_lowres, cv::COLOR_RGBA2BGR);
        std::cout << "Converted RGBA image to BGR format ... ";
    } else if (im_lowres.channels() == 1) {
        cv::cvtColor(im_lowres, im_lowres, cv::COLOR_GRAY2BGR);
        std::cout << "Converted grayscale image to BGR format ... ";
    }

    std::cout << " ok" << std::endl;
}

bool Maze::isFree(float fx, float fy, bool lowres) const
{
    int x = static_cast<int>(std::round(fx));
    int y = static_cast<int>(std::round(fy));

    const cv::Mat& selected_im = lowres ? im_lowres : im;

    if (x < 0 || y < 0 || x >= selected_im.cols || y >= selected_im.rows) {
        return false;
    }

    cv::Vec3b color = selected_im.at<cv::Vec3b>(y, x);

    if (color[0] == color[1] && color[1] == color[2]) // if the color is gray and darker than 120
    {
        if (color[0] < permanent_obstacle_treshold) {
            return false;
        }
    } else if (color[0] == 0 && color[1] > 0 && color[2] == 0) // if the color is green
    {
        return false;
    }

    return true;
}


bool Maze::isFree(const Point& p, bool lowres) const { return isFree(p.x, p.y); }

void Maze::passThrough(int x, int y) { path.push_back(cv::Point(x, y)); }

void Maze::display(const std::string& name, const std::string& type)
{
    cv::Mat& imageToShow = (type == "im") ? im : (type == "out") ? out : im; // Default to `im` if invalid input

    if (std::find(windows.begin(), windows.end(), name) == windows.end()) {
        windows.push_back(name);
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::resizeWindow(name, 1000, (1000 * imageToShow.rows) / imageToShow.cols);
    }
    cv::imshow(name, imageToShow);
}


void Maze::write(int x, int y, int r, int g, int b, bool show)
{
    out.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
    if (show) {
        display("Maze", "out");
        cv::waitKey(1);
    }
}

void Maze::save()
{
    cv::imwrite(mazeFile(), im);
    display("Maze", "im");
}


void Maze::saveSolution(std::string suffix, const std::vector<Point>& astar_path, const std::vector<Point>& eb_path_smoothed, const std::vector<Obstacle>& obstacles)
{
    out = im.clone();

    cv::Vec3b colour_astar(255, 50, 50);
    cv::Vec3b colour_astar_points(80, 0, 0);

    cv::Vec3b colour_eb(50, 50, 255);
    cv::Vec3b colour_eb_points(0, 0, 80);

    cv::Vec3b colour_eb_smoothed(200, 200, 0);
    cv::Vec3b colour_eb_smoothed_points(80, 80, 0);

    const int scale              = 30; // Scale factor: Each grid cell becomes 'scale' x 'scale' pixels
    const int lineThickness      = 20;
    const int circleThickness    = lineThickness + 2;
    const int rectangleThickness = lineThickness + 2;

    cv::Mat highResOut;
    cv::resize(out, highResOut, cv::Size(im.cols * scale, im.rows * scale), 0, 0, cv::INTER_NEAREST);

    renderObstacles(obstacles, highResOut, scale);

    auto drawLine      = [&](const cv::Point& p1, const cv::Point& p2, const cv::Vec3b& color) { cv::line(highResOut, p1, p2, color, lineThickness); };
    auto drawPoint     = [&](const cv::Point& p, const cv::Vec3b& color) { cv::circle(highResOut, p, circleThickness / 2, color, cv::FILLED); };
    auto drawRectangle = [&](const cv::Point& p1, const cv::Vec3b& color) {
        cv::rectangle(highResOut, cv::Point(p1.x - rectangleThickness / 2, p1.y - rectangleThickness / 2),
                      cv::Point(p1.x + rectangleThickness / 2, p1.y + rectangleThickness / 2), color, cv::FILLED);
    };

    for (size_t i = 0; i < astar_path.size() - 1; ++i) {
        cv::Point p1(std::round(astar_path[i].x * scale), std::round(astar_path[i].y * scale));
        cv::Point p2(std::round(astar_path[i + 1].x * scale), std::round(astar_path[i + 1].y * scale));
        drawLine(p1, p2, colour_astar);
        drawPoint(p1, colour_astar_points);
    }

    for (size_t i = 0; i < path_eb.size() - 1; ++i) {
        cv::Point p1(std::round(path_eb[i].x * scale), std::round(path_eb[i].y * scale));
        cv::Point p2(std::round(path_eb[i + 1].x * scale), std::round(path_eb[i + 1].y * scale));
        drawLine(p1, p2, colour_eb);
        drawPoint(p1, colour_eb_points);
    }

    for (size_t i = 0; i < eb_path_smoothed.size() - 1; ++i) {
        cv::Point p1(std::round(eb_path_smoothed[i].x * scale), std::round(eb_path_smoothed[i].y * scale));
        cv::Point p2(std::round(eb_path_smoothed[i + 1].x * scale), std::round(eb_path_smoothed[i + 1].y * scale));
        drawLine(p1, p2, colour_eb_smoothed);
        drawPoint(p1, colour_eb_smoothed_points);
    }

    int dot          = filename.find(".");
    std::string name = filename.substr(0, dot) + "_" + suffix + ".png";
    if (!cv::imwrite(name, highResOut)) {
        std::cerr << "Error: Failed to save image to " << name << std::endl;
    } else {
        std::cout << "Saved solution to " << name << std::endl;
    }
    cv::namedWindow("Solution", cv::WINDOW_NORMAL);
    cv::resizeWindow("Solution", 1500, 1200);
    cv::moveWindow("Solution", 100, 100);
    cv::imshow("Solution", highResOut);
}

Point Maze::getStart() const { return start_; }

Point Maze::getGoal() const { return goal_; }

Point Maze::findStart()
{
    Point start(-1, -1);
    for (int y = 0; y < im.rows; ++y) {
        for (int x = 0; x < im.cols; ++x) {
            cv::Vec3b color = im.at<cv::Vec3b>(y, x);
            if (color[0] == 0 && color[1] == 0 && color[2] == 255) // blue
            {                                                      
                start = Point(x, y);
                std::cout << "Start found (blue): (" << start.x << ", " << start.y << ")\n";
                start_ = start;
                return start;
            }
        }
    }
    throw std::runtime_error("Failed to find the start point in the maze");
}

Point Maze::findGoal()
{
    Point goal(-1, -1);
    for (int y = 0; y < im.rows; ++y) {
        for (int x = 0; x < im.cols; ++x) {
            cv::Vec3b color = im.at<cv::Vec3b>(y, x);
            if (color[0] == 255 && color[1] == 0 && color[2] == 0) // red
            {                                                      
                goal = Point(x, y);
                std::cout << "Goal found (red): (" << goal.x << ", " << goal.y << ")\n";
                goal_ = goal;
                return goal;
            }
        }
    }
    throw std::runtime_error("Failed to find the goal point in the maze");
}

void Maze::setStart(const Point& start) { start_ = start; }

void Maze::setGoal(const Point& goal) { goal_ = goal; }

void Maze::setElasticBandPath(const std::vector<Point>& elasticBandPath)
{
    path_eb.clear();
    for (const auto& pos : elasticBandPath) {
        path_eb.emplace_back(pos.x, pos.y);
    }
}
void Maze::renderObstacles(const std::vector<Obstacle>& obstacles, cv::Mat& image, int scale)
{
    // Iterate over each obstacle
    for (const auto& obstacle : obstacles) {
        // If the obstacle is movable, we should first remove its previous position
        if (obstacle.getType() == Obstacle::MOVABLE) {
            // Reset the previous position of the obstacle
            for (int y = (obstacle.getYPrev() - obstacle.getHeight() / 2) * scale; y < (obstacle.getYPrev() + obstacle.getHeight() / 2) * scale; ++y) {
                for (int x = (obstacle.getXPrev() - obstacle.getWidth() / 2) * scale; x < (obstacle.getXPrev() + obstacle.getWidth() / 2) * scale; ++x) {
                    // Ensure the coordinates are within bounds of the image
                    if (y >= 0 && y < image.rows && x >= 0 && x < image.cols) {
                        // Restore the pixel from the original image (background)
                        image.at<cv::Vec3b>(y, x) = original_im.at<cv::Vec3b>(y / scale, x / scale);
                    }
                }
            }
        }

        // Render the obstacle at its current position (scaled)
        for (int y = (obstacle.getY() - obstacle.getHeight() / 2) * scale; y < (obstacle.getY() + obstacle.getHeight() / 2) * scale; ++y) {
            for (int x = (obstacle.getX() - obstacle.getWidth() / 2) * scale; x < (obstacle.getX() + obstacle.getWidth() / 2) * scale; ++x) {
                // Ensure the coordinates are within bounds of the image
                if (y >= 0 && y < image.rows && x >= 0 && x < image.cols) {
                    if (obstacle.isActive()) {
                        // Render the obstacle with its color if it's active (scaled)
                        image.at<cv::Vec3b>(y, x) = cv::Vec3b(obstacle.getColor()[0], obstacle.getColor()[1], obstacle.getColor()[2]);
                    } else {
                        // If the obstacle is not active, reset the pixel to the original image (background)
                        image.at<cv::Vec3b>(y, x) = original_im.at<cv::Vec3b>(y / scale, x / scale);
                    }
                }
            }
        }
    }
}


} // namespace ecn
