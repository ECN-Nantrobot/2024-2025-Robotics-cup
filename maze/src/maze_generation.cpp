#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <position.h>
#include <maze.h>
#include <a_star.h>
#include <elastic_bands.h>
#include <point.h>

struct Node
{
    int x, y;       // Node position
    char c;         // Character to be displayed (' ' for paths, '#' for walls)
};

Node *nodes;        // Nodes array
int width_maze, height_maze;  // Maze dimensions

void initializeEmptyMaze()
{
    // Allocate memory for maze
    nodes = (Node *)(calloc(width_maze * height_maze, sizeof(Node)));
    if (nodes == NULL)
    {
        fprintf(stderr, "Error: Out of memory!\n");
        exit(1);
    }

    // Initialize maze with all empty spaces and a black border
    for (int i = 0; i < height_maze; i++)
    {
        for (int j = 0; j < width_maze; j++)
        {
            Node *n = nodes + j + i * width_maze;
            n->x = j;
            n->y = i;

            // Set border walls
            if (i == 0 || i == height_maze - 1 || j == 0 || j == width_maze - 1)
            {
                n->c = '#'; // Border
            }
            else
            {
                n->c = ' '; // Empty space
            }
        }
    }
}

void placeObstacle(int centerX, int centerY, int width, int height, int border)
{
    int startX = centerX - width/2 - border;
    int endX = centerX + width/2 + border;
    int startY = centerY - height/2 - border;
    int endY = centerY + height/2 + border;

    // Clamp obstacle bounds to fit within the maze
    startX = std::max(1, startX);
    endX = std::min(width_maze - 2, endX);
    startY = std::max(1, startY);
    endY = std::min(height_maze - 2, endY);

    for (int i = startY; i <= endY; i++){
        for (int j = startX; j <= endX; j++){

            Node *n = nodes + j + i * width_maze;

            if (i < startY + border || i > endY - border || j < startX + border || j > endX - border){
                n->c = 'B'; // Use 'B' to mark border cells (grey)
            }
            else{
                n->c = '#'; // Internal cells are walls (black)
            }
        }
    }
}


void drawMazeToImage(const std::string &filename)
{
    // Create an OpenCV image with a white background
    cv::Mat image(height_maze, width_maze, CV_8UC3, cv::Scalar(255, 255, 255)); // White background

    // Fill the image based on the maze structure
    for (int i = 0; i < height_maze; i++)
    {
        for (int j = 0; j < width_maze; j++)
        {
            Node *n = nodes + j + i * width_maze;

            if (n->c == '#')
            {
                // Draw internal walls (black)
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
            else if (n->c == 'B')
            {
                // Draw borders (grey)
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(150, 150, 150);
            }
        }
    }

    // Specify the full path to the "maze" folder
    std::string fullPath = "/home/ecn/ecn_arpro/maze/mazes/" + filename;
    std::string fullPath2 = "/media/sf_SharedFolderVBox/" + filename;

    // Save the maze as an image
    if (!cv::imwrite(fullPath, image))
    {
        fprintf(stderr, "Error: Could not save image to %s!\n", fullPath.c_str());
        exit(1);
    }
    if (!cv::imwrite(fullPath2, image))
    {
        fprintf(stderr, "Error: Could not save image to %s!\n", fullPath2.c_str());
        exit(1);
    }

    std::cout << "Maze saved as image: " << fullPath << std::endl;



    // Display the generated image
    cv::Mat maze_image = cv::imread(fullPath);

    if (maze_image.empty()){
        std::cerr << "Error: Could not load the maze image!" << std::endl;
    }

    // Create a resizable window
    cv::namedWindow("Generated Maze", cv::WINDOW_NORMAL);
    int scale = 6;
    cv::resizeWindow("Generated Maze", maze_image.cols * scale, maze_image.rows * scale);

    cv::imshow("Generated Maze", maze_image);
    cv::waitKey(0);
}


int main()
{
    // Set maze dimensions (must be odd to ensure proper border handling)
    width_maze = 61;
    height_maze = 41;

    // Initialize an empty maze with a black border
    initializeEmptyMaze();

    // Place rectangle obstacle at x,y with hight,width and border
    placeObstacle(10, 20, 10, 5, 1);
    placeObstacle(20, 35, 10, 2, 2);
    placeObstacle(30, 20, 12, 8, 2);
    placeObstacle(40, 4, 4, 9, 3);

    // Save and display the maze
    std::string outputFile = "maze_generated.png";
    drawMazeToImage(outputFile);

    // Free allocated memory
    free(nodes);

    return 0;
}
