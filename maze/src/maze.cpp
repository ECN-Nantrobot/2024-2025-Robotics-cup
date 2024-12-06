#include "maze.h"
#include "point.h"

namespace ecn
{

    std::string Maze::mazeFile(const std::string &file)
    {
        return std::string(MAZES) + "/" + file;
    }

    Maze::Maze() {}

    void Maze::load(std::string _filename)
    {
        std::cout << "Loading " << _filename << " ... ";
        filename = _filename;

        im = cv::imread(filename, cv::IMREAD_UNCHANGED);

        if (im.empty())
        {
            throw std::runtime_error("Failed to load the image at: " + filename);
        }

        if (im.channels() == 4)
        {
            cv::cvtColor(im, im, cv::COLOR_RGBA2BGR);
            std::cout << "Converted RGBA image to BGR format";
        }
        else if (im.channels() == 1)
        {
            cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
            std::cout << "Converted grayscale image to BGR format";
        }
        out = im.clone();

        std::cout << " ok" << std::endl;
    }

    bool Maze::isFree(float fx, float fy) const
    {
        int x = static_cast<int>(std::round(fx));
        int y = static_cast<int>(std::round(fy));

        if (x < 0 || y < 0 || x >= im.cols || y >= im.rows)
        {
            return false;
        }

        cv::Vec3b color = im.at<cv::Vec3b>(y, x);

        if (color[0] == color[1] && color[1] == color[2])
        {
            if (color[0] < 255)
            {
                return false;
            }
        }

        return true;
    }

    bool Maze::isFree(const Point &p) const
    {
        return isFree(p.x, p.y);
    }

    void Maze::passThrough(int x, int y)
    {
        path.push_back(cv::Point(x, y));
    }

    void Maze::display(const std::string &name, const cv::Mat &im)
    {
        if (std::find(windows.begin(), windows.end(), name) == windows.end())
        {
            windows.push_back(name);
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, 1000, (1000 * im.rows) / im.cols);
        }
        cv::imshow(name, im);
    }

    void Maze::write(int x, int y, int r, int g, int b, bool show)
    {
        out.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        if (show)
        {
            display("Maze", out);
            cv::waitKey(1);
        }
    }

    void Maze::save()
    {
        cv::imwrite(mazeFile(), im);
        display("Maze", im);
    }

    void Maze::saveSolution(std::string suffix)
    {
        cv::Vec3b colour_astar(255, 0, 0); // Color for A* path
        cv::Vec3b colour_eb(0, 255, 0);    // Color for Elastic Band path

        // Draw the A* path
        for (const auto &point : path)
        {
            out.at<cv::Vec3b>(point) = colour_astar;
        }

        // Draw the Elastic Band path
        for (const auto &point : path_eb)
        {
            out.at<cv::Vec3b>(point) = colour_eb;
        }

        // Ensure obstacles remain correctly represented
        for (int x = 0; x < im.cols; ++x)
        {
            for (int y = 0; y < im.rows; ++y)
            {
                if (!isFree(x, y))
                {
                    write(x, y, 0, 0, 0, false); // Set obstacles to black
                }
            }
        }

        int dot = filename.find(".");
        std::string name = filename.substr(0, dot) + "_" + suffix + ".png";
        cv::imwrite(mazeFile(name), out);
        cv::imshow("Solution", out);
    }

    Point Maze::getStart() const
    {
        return start_;
    }

    Point Maze::getGoal() const
    {
        return goal_;
    }

    Point Maze::findStart()
    {
        Point start(-1, -1);
        for (int y = 0; y < im.rows; ++y)
        {
            for (int x = 0; x < im.cols; ++x)
            {
                cv::Vec3b color = im.at<cv::Vec3b>(y, x);
                if (color[0] == 255 && color[1] == 0 && color[2] == 0)
                { // Red color
                    start = Point(x, y);
                    std::cout << "Start: (" << start.x << ", " << start.y << ")\n";
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
        for (int y = 0; y < im.rows; ++y)
        {
            for (int x = 0; x < im.cols; ++x)
            {
                cv::Vec3b color = im.at<cv::Vec3b>(y, x);
                if (color[0] == 0 && color[1] == 0 && color[2] == 255)
                { // Blue color
                    goal = Point(x, y);
                    std::cout << "Goal: (" << goal.x << ", " << goal.y << ")\n";
                    goal_ = goal;
                    return goal;
                }
            }
        }
        throw std::runtime_error("Failed to find the goal point in the maze");
    }

    Point Maze::findCornerStart()
    {
        for (int y = 0; y < im.rows; ++y)
        {
            for (int x = 0; x < im.cols; ++x)
            {
                if (isFree(x, y))
                {
                    Point start(x, y);
                    std::cout << "Corner Start: (" << start.x << ", " << start.y << ")\n";
                    start_ = start;
                    return start;
                }
            }
        }
        throw std::runtime_error("Failed to find a corner start in the maze");
    }

    Point Maze::findCornerGoal()
    {
        for (int y = im.rows - 1; y >= 0; --y)
        {
            for (int x = im.cols - 1; x >= 0; --x)
            {
                if (isFree(x, y))
                {
                    Point goal(x, y);
                    std::cout << "Corner Goal: (" << goal.x << ", " << goal.y << ")\n";
                    goal_ = goal;
                    return goal;
                }
            }
        }
        throw std::runtime_error("Failed to find a corner goal in the maze");
    }

    void Maze::setStart(const Point &start)
    {
        start_ = start;
    }

    void Maze::setGoal(const Point &goal)
    {
        goal_ = goal;
    }

    void Maze::setElasticBandPath(const std::vector<Point> &elasticBandPath)
    {
        path_eb.clear();
        for (const auto &pos : elasticBandPath)
        {
            path_eb.emplace_back(pos.x, pos.y);
        }
    }

} // namespace ecn
