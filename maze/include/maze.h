#ifndef MAZE_H
#define MAZE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <point.h>

namespace ecn {

typedef std::pair<int, int> Pair;

class Maze
{
public:

//    Maze(const cv::Mat &image) : im(image) {
//        // Precompute free cells
//        freeCells.resize(im.rows, std::vector<bool>(im.cols, false));
//        for (int y = 0; y < im.rows; ++y) {
//            const uchar *row = im.ptr<uchar>(y); // Pointer to image row for faster access
//            for (int x = 0; x < im.cols; ++x) {
//                freeCells[y][x] = (row[x] != 0);
//            }
//        }
//    }

//    // Lookup method for isFree
//    bool isFree(int x, int y) const {
//        if (x < 0 || y < 0 || x >= im.cols || y >= im.rows)
//            return false; // Bounds check
//        return freeCells[y][x];
//    }



  static std::string mazeFile(const std::string &file = "maze.png")
  {
    std::string abs_file = MAZES;
    return abs_file + "/" + file;
  }

  Maze() {}

  Maze(std::string _filename)
  {
    load(_filename);
  }

  void load(std::string _filename)
  {
      std::cout << "Loading " << _filename << " ...";
      filename = _filename;
      im = cv::imread(filename, cv::IMREAD_GRAYSCALE);

      // Ensure non-white pixels are black
      for (int y = 0; y < im.rows; ++y)
      {
          for (int x = 0; x < im.cols; ++x)
          {
              if (im.at<uchar>(y, x) != 255)  // If the pixel is not white
              {
                  im.at<uchar>(y, x) = 0;    // Set it to black
              }
          }
      }

      cv::cvtColor(im, out, cv::COLOR_GRAY2BGR); // Convert to color for visualization
      std::cout << " ok" << std::endl;
  }

  Maze(int height, int width)
  {
    im = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
  }

  bool isFree(int x, int y) const
  {
    if(x < 0 || y < 0 || x >= im.cols || y >= im.rows)
      return 0;
    return im.at<uchar>(y,x);
  }

  inline bool isFree(const Point &p) const
  {
    return isFree(p.x, p.y);
  }

  inline int height() const {return im.rows;}
  inline int width() const {return im.cols;}


  ecn::Point start()
  {
      Point ret;
      for(ret.y=0;ret.y<im.rows; ++ret.y)
      {
          for(ret.x=0;ret.x<im.cols;++ret.x)
          {
              if(isFree(ret.x, ret.y))
              {
                  std::cout << "Start @ (" << ret.x << ", " << ret.y << ")\n";
                  return ret;
              }
          }
      }
      return ret;
  }

  ecn::Point end()
  {
      Point ret;
      for(ret.y=im.rows-1;ret.y> 0; --ret.y)
      {
          for(ret.x=im.cols-1;ret.x>0;--ret.x)
          {
              if(isFree(ret.x, ret.y))
              {
                  //std::cout << "End @ (" << ret.x << ", " << ret.y << ")\n";
                  return ret;
              }
          }
      }
      return ret;
  }


  void passThrough(int x, int y)
  {
    path.push_back(cv::Point(x,y));
  }

  void dig(int x, int y)
  {
    im.at<uchar>(y,x) = 255;
  }

  void display(const std::string &name, const cv::Mat &im)
  {
    if(std::find(windows.begin(), windows.end(), name) == windows.end())
    {
      windows.push_back(name);
      cv::namedWindow(name, cv::WINDOW_NORMAL);
      cv::resizeWindow(name, 1000, (1000*im.rows)/im.cols);
    }
    cv::imshow(name, im);
  }

  void write(int x, int y, int r=0, int g=0, int b=0, bool show = true)
  {
    out.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
    if(show)
    {
      display("Maze", out);
      cv::waitKey(1);
    }
  }

  void save()
  {
    cv::imwrite(mazeFile(), im);
    display("Maze", im);
  }

  void saveSolution(std::string suffix, const std::vector<cv::Point>& elasticBandPath)
  {
    cv::cvtColor(im, out, cv::COLOR_GRAY2BGR);
    cv::Vec3b colour_astar(255, 0, 0); // Blue Green Red 

    colour_astar[1] = 0;
    for(int i = 0; i < path.size(); ++i)
    {
      //colour from blue to red
      // colour_astar[2] = i*255/path.size(); 
      // colour_astar[0] = 255-colour_astar[2];
      out.at<cv::Vec3b>(path[i]) = colour_astar;
    }


    // draw elastic band path in green
    for(const auto& p : elasticBandPath)
    {
      out.at<cv::Vec3b>(p) = cv::Vec3b(0, 255, 0); // Blue Green Red 
    }


    // re-write black nodes just to be sure...
    for(int x = 0; x < im.cols; ++x)
    {
      for(int y = 0; y < im.rows; ++y)
      {
        if(!isFree(x,y))
          write(x,y,0,0,0,false);
      }
    }

    int dot = filename.find(".");
    std::string name = filename.substr(0, dot) + "_" + suffix + ".png";
    cv::imwrite(mazeFile(name), out);
    display("Solution", out);
  }



  const cv::Mat& getIm() const { return im; }
  const cv::Mat& getOut() const { return out; }

  // Provide setter methods if needed
  void setIm(const cv::Mat& image) { im = image; }
  void setOut(const cv::Mat& output) { out = output; }

protected:
  cv::Mat im, out;
  std::string filename;
  std::vector<cv::Point> path;
  std::vector<std::string> windows;
};
}

#endif

