#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include <maze.h>


struct Obstacle {
    float x, y, width, height;
    sf::RectangleShape shape;
};

class MazeEditor {
public:
    MazeEditor()
        : window(sf::VideoMode(1, 1), "Maze Editor") {

        mazeWidth = 200;
        mazeHeight = 100;

        double format = static_cast<double>(mazeHeight) / mazeWidth;

        int window_height = 500;
        int window_width = static_cast<int>(window_height / format);

        gridSize = window_height / mazeHeight;

        borderSize = 10;

        gridColor = sf::Color(250, 250, 250);
        obstacleColor = sf::Color::Black;
        borderColor = sf::Color::Black;  // Black border color
        Startcolour = sf::Color::Blue;
        Endcolour = sf::Color::Red;

        // Initialize the first click position
        firstClick = sf::Vector2i(-1, -1);

        window.create(sf::VideoMode(window_width, window_height), "Maze Editor");

        // Initialize render texture to save the maze as an image
        renderTexture.create(mazeWidth, mazeHeight);
    }

    void run() {
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }

                // First click to set the starting point of the rectangle
                if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                    if (firstClick.x == -1 && firstClick.y == -1) {
                        firstClick = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);  // Store first click
                    } else {
                        placeObstacle(event.mouseButton.x, event.mouseButton.y);  // Second click, place the rectangle
                        renderMazeToImage(filename_maze);  // Save the maze image
                        firstClick = sf::Vector2i(-1, -1);  // Reset the first click after drawing the rectangle
                    }
                }
                else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                    placeStartEnd(event.mouseButton.x, event.mouseButton.y);  // Place the obstacle on right click    
                    renderMazeToImage(filename_maze);
                }
            }

            window.clear();
            drawGrid();  // Draw the grid based on maze dimensions
            drawObstacles(false);  // Draw obstacles on the grid
            drawRectangle();  // Draw the rectangle if two clicks are made
            drawBorder(false);
            window.display();
        }
    }

private:
    void drawGrid() {
        // Draw the grid background (cells) based on maze dimensions (for display purposes)
        for (int i = 0; i < mazeWidth; ++i) {
            for (int j = 0; j < mazeHeight; ++j) {
                sf::RectangleShape cell(sf::Vector2f(gridSize, gridSize));
                cell.setPosition(i * gridSize, j * gridSize);
                cell.setFillColor(gridColor);
                window.draw(cell);
            }
        }

        // Draw the grid lines (borders) based on maze dimensions (only for window display, not saved)
        sf::VertexArray lines(sf::Lines);
        for (int i = 0; i <= mazeWidth; ++i) {
            lines.append(sf::Vertex(sf::Vector2f(i * gridSize, 0), sf::Color::Black));
            lines.append(sf::Vertex(sf::Vector2f(i * gridSize, mazeHeight * gridSize), sf::Color::Black));
        }
        for (int j = 0; j <= mazeHeight; ++j) {
            lines.append(sf::Vertex(sf::Vector2f(0, j * gridSize), sf::Color::Black));
            lines.append(sf::Vertex(sf::Vector2f(mazeWidth * gridSize, j * gridSize), sf::Color::Black));
        }
        window.draw(lines);
    }

    void drawObstacles(bool print) {
        if(print){
            for (const auto& obstacle : obstacles_print) {
                renderTexture.draw(obstacle.shape);
                renderTexture.draw(Start.shape);
                renderTexture.draw(End.shape);
            }
        }
        else {
            for (const auto& obstacle : obstacles) {
                window.draw(obstacle.shape);
                window.draw(Start.shape);
                window.draw(End.shape);
            }
        }   
    }


    void placeObstacle(int x, int y) {
        // Convert mouse position into grid coordinates in the window
        int gridX = x / gridSize;
        int gridY = y / gridSize;

        Obstacle newObstacle;
        newObstacle.x = firstClick.x / gridSize;
        newObstacle.y = firstClick.y / gridSize;
        newObstacle.width = gridX - newObstacle.x +1;
        newObstacle.height = gridY - newObstacle.y +1;

        // Ensure the rectangle has positive width and height
        if (newObstacle.width < 0) {
            newObstacle.width = -newObstacle.width;
            newObstacle.x = gridX;
        }
        if (newObstacle.height < 0) {
            newObstacle.height = -newObstacle.height;
            newObstacle.y = gridY;
        }

        newObstacle.shape.setSize(sf::Vector2f(newObstacle.width * gridSize, newObstacle.height * gridSize));
        newObstacle.shape.setPosition(newObstacle.x * gridSize, newObstacle.y * gridSize);
        newObstacle.shape.setFillColor(obstacleColor);
        obstacles.push_back(newObstacle);
        
        newObstacle.shape.setSize(sf::Vector2f(newObstacle.width, newObstacle.height));
        newObstacle.shape.setPosition(newObstacle.x, newObstacle.y);
        obstacles_print.push_back(newObstacle);
        
        renderTexture.draw(newObstacle.shape);
        renderTexture.display();
    }

    void placeStartEnd(int x, int y) {
        // Convert mouse position into grid coordinates in the window
        int gridX = x / gridSize;
        int gridY = y / gridSize;

        if (!start_placed) {
            // Place Start obstacle (blue color)
            Start.x = gridX;
            Start.y = gridY;
            Start.width = 1;
            Start.height = 1;

            Start.shape.setSize(sf::Vector2f(Start.width * gridSize, Start.height * gridSize));
            Start.shape.setPosition(Start.x * gridSize, Start.y * gridSize);
            Start.shape.setFillColor(Startcolour);

            obstacles_print.push_back(Start);  // Add Start to obstacles to render
            start_placed = true;  // Mark Start as placed

        } else {
            // Place End obstacle (red color)
            End.x = gridX;
            End.y = gridY;
            End.width = 1;
            End.height = 1;

            End.shape.setSize(sf::Vector2f(End.width * gridSize, End.height * gridSize));
            End.shape.setPosition(End.x * gridSize, End.y * gridSize);
            End.shape.setFillColor(Endcolour);

            obstacles_print.push_back(End);  // Add End to obstacles to render
            start_placed = false;  // Mark End as placed, reset for next cycle
        }

        // Optional: You can also draw Start and End obstacles directly to renderTexture if required
        // renderTexture.draw(Start.shape);
        // renderTexture.draw(End.shape);
        renderTexture.display();
    }


    void drawRectangle() {
        if (firstClick.x != -1 && firstClick.y != -1) {
            // If first click exists, draw a rectangle on the screen as the user selects it
            sf::RectangleShape rectangle(sf::Vector2f(0, 0));
            int width = (sf::Mouse::getPosition(window).x - firstClick.x) / gridSize;
            int height = (sf::Mouse::getPosition(window).y - firstClick.y) / gridSize;
            rectangle.setSize(sf::Vector2f(width * gridSize, height * gridSize));
            rectangle.setPosition(firstClick.x, firstClick.y);
            rectangle.setOutlineColor(sf::Color::Blue);
            rectangle.setOutlineThickness(2);
            rectangle.setFillColor(sf::Color(0, 0, 255, 50));  // Transparent blue fill
            window.draw(rectangle);
        }
    }

    void drawBorder(bool print) {
        // Draw a black border around the maze (scaled by gridSize)
        int gridSize_temp = gridSize;
        if(print){int gridSize_temp = 1;}
        
        sf::RectangleShape border(sf::Vector2f((mazeWidth - 2*borderSize) *gridSize_temp, (mazeHeight - 2*borderSize) *gridSize_temp));
        border.setPosition(0 + borderSize *gridSize_temp, 0 + borderSize *gridSize_temp);
        border.setOutlineColor(borderColor);
        border.setOutlineThickness(borderSize * gridSize_temp);  
        border.setFillColor(sf::Color::Transparent);
        window.draw(border);
        renderTexture.draw(border);
    }

    void renderMazeToImage(const std::string& filename) {
        // Clear the render texture and draw only the maze (without grid)
        renderTexture.clear(sf::Color::White);  // Clear with a white background (or transparent if needed)

        // Draw obstacles to the render texture (only obstacles, no grid lines)
        drawObstacles(true);
        drawBorder(true);

        renderTexture.display();  // Display the rendered content to the texture

        // Get the maze's texture (without grid lines)
        sf::Texture texture = renderTexture.getTexture();  // Get texture from render texture
        texture.copyToImage().saveToFile(filename);  // Save the image to a file
    }

    Obstacle Start;
    Obstacle End;
    sf::RenderWindow window;
    sf::RenderTexture renderTexture;
    int gridSize;
    int mazeWidth, mazeHeight;
    sf::Color gridColor;
    sf::Color obstacleColor;
    sf::Color borderColor, Startcolour, Endcolour;  // Border color
    int borderSize;
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle> obstacles_print;
    std::string filename_maze = ecn::Maze::mazeFile("maze_generated_interactive.png");
    bool start_placed = false;

    sf::Vector2i firstClick; // Position of the first click (starting point for the rectangle)
};

int main() {
    MazeEditor editor;
    editor.run();

    return 0;
}
