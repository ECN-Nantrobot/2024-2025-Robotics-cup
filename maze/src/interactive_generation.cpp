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

        //PARAMETERS TO SET
        maze_width = 301;
        maze_height = 201;
        grid_size = std::max(6, std::min(45, 700 / std::max(maze_width, maze_height))); //automatically set grid size based on maze dimensions(for visualization)

        border_size = 1;
        grid_color = sf::Color::White;
        obstacle_color = sf::Color::Black;
        border_color = sf::Color::Black;
        startcolour = sf::Color::Blue;
        endcolour = sf::Color::Red;

        first_click = sf::Vector2i(-1, -1); // Initialize the first click position

        window.create(sf::VideoMode(maze_width*grid_size, maze_height*grid_size), "Maze Editor");

        renderTexture.create(maze_width, maze_height); // Initialize render texture to save the maze as an image
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
                    if (first_click.x == -1 && first_click.y == -1) {
                        first_click = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);  // Store first click
                    } else {
                        placeObstacle(event.mouseButton.x, event.mouseButton.y);  // Second click, place the rectangle
                        renderMazeToImage(filename_maze);
                        first_click = sf::Vector2i(-1, -1);  // Reset the first click after drawing the rectangle
                    }
                }
                else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                    placeStartEnd(event.mouseButton.x, event.mouseButton.y);  // Place the obstacle on right click    
                    renderMazeToImage(filename_maze);
                }
            }

            window.clear();
            drawGrid();  // Draw the grid based on maze dimensions
            drawBorder();  // Draw the border around the maze
            drawObstacles(false);  // Draw obstacles on the grid
            drawRectangle();  // Draw the rectangle if two clicks are made
            window.display();
        }
    }

private:
    void drawGrid() {
        // Draw the grid background (cells) based on maze dimensions (for display purposes)
        for (int i = 0; i < maze_width; ++i) {
            for (int j = 0; j < maze_height; ++j) {
                sf::RectangleShape cell(sf::Vector2f(grid_size, grid_size));
                cell.setPosition(i * grid_size, j * grid_size);
                cell.setFillColor(grid_color);
                window.draw(cell);
            }
        }

        // Draw the grid lines (borders) based on maze dimensions (only for window display, not saved)
        sf::VertexArray lines(sf::Lines);
        for (int i = 0; i <= maze_width; ++i) {
            lines.append(sf::Vertex(sf::Vector2f(i * grid_size, 0), sf::Color::Black));
            lines.append(sf::Vertex(sf::Vector2f(i * grid_size, maze_height * grid_size), sf::Color::Black));
        }
        for (int j = 0; j <= maze_height; ++j) {
            lines.append(sf::Vertex(sf::Vector2f(0, j * grid_size), sf::Color::Black));
            lines.append(sf::Vertex(sf::Vector2f(maze_width * grid_size, j * grid_size), sf::Color::Black));
        }
        window.draw(lines);
    }

    void drawObstacles(bool print) {
        if(print){
            for (const auto& obstacle : obstacles_print) {
                renderTexture.draw(obstacle.shape);
            }
            renderTexture.draw(start_print.shape);
            renderTexture.draw(end_print.shape);
            renderTexture.draw(border_print);
        }
        else {
            for (const auto& obstacle : obstacles) {
                window.draw(obstacle.shape);
            }
            window.draw(start.shape);
            window.draw(end.shape);
            window.draw(border);
        }   
    }

    void placeObstacle(int x, int y) {
        // Convert mouse position into grid coordinates in the window
        int grid_x = x / grid_size;
        int grid_y = y / grid_size;

        Obstacle newObstacle;
        newObstacle.x = first_click.x / grid_size;
        newObstacle.y = first_click.y / grid_size;
        newObstacle.width = grid_x - newObstacle.x +1;
        newObstacle.height = grid_y - newObstacle.y +1;

        // Ensure the rectangle has positive width and height
        if (newObstacle.width < 0) {
            newObstacle.width = -newObstacle.width;
            newObstacle.x = grid_x;
        }
        if (newObstacle.height < 0) {
            newObstacle.height = -newObstacle.height;
            newObstacle.y = grid_y;
        }

        newObstacle.shape.setSize(sf::Vector2f(newObstacle.width * grid_size, newObstacle.height * grid_size));
        newObstacle.shape.setPosition(newObstacle.x * grid_size, newObstacle.y * grid_size);
        newObstacle.shape.setFillColor(obstacle_color);
        obstacles.push_back(newObstacle);
        
        newObstacle.shape.setSize(sf::Vector2f(newObstacle.width, newObstacle.height));
        newObstacle.shape.setPosition(newObstacle.x, newObstacle.y);
        obstacles_print.push_back(newObstacle);
    }

    void placeStartEnd(int x, int y) {
        int grid_x = x / grid_size;
        int grid_y = y / grid_size;

        if (!start_placed) {
            // Place start obstacle (blue color)
            start.x = x / grid_size;
            start.y = y / grid_size;
            start.width = grid_x - start.x+1;
            start.height = grid_y - start.y+1;

            start.shape.setSize(sf::Vector2f(start.width * grid_size, start.height * grid_size));
            start.shape.setPosition(start.x * grid_size, start.y * grid_size);
            start.shape.setFillColor(startcolour);

            start_print = start;
            start_print.shape.setSize(sf::Vector2f(start.width, start.height));
            start_print.shape.setPosition(start.x, start.y);
            
            start_placed = true; 

        } else {
            // Place end obstacle (red color)
            end.x = x / grid_size;
            end.y = y / grid_size;
            end.width = 1;
            end.height = 1;

            end.shape.setSize(sf::Vector2f(end.width * grid_size, end.height * grid_size));
            end.shape.setPosition(end.x * grid_size, end.y * grid_size);
            end.shape.setFillColor(endcolour);

            end_print = end;
            end_print.shape.setSize(sf::Vector2f(end.width, end.height));
            end_print.shape.setPosition(end.x, end.y);

            start_placed = false;
        }
    }

    void drawRectangle() {
        if (first_click.x != -1 && first_click.y != -1) {
            // If first click exists, draw a rectangle on the screen as the user selects it
            sf::RectangleShape rectangle(sf::Vector2f(0, 0));
            int width = (sf::Mouse::getPosition(window).x - first_click.x) / grid_size;
            int height = (sf::Mouse::getPosition(window).y - first_click.y) / grid_size;
            rectangle.setSize(sf::Vector2f(width * grid_size, height * grid_size));
            rectangle.setPosition(first_click.x, first_click.y);
            rectangle.setOutlineColor(sf::Color::Blue);
            rectangle.setOutlineThickness(2);
            rectangle.setFillColor(sf::Color(0, 0, 255, 50));  // Transparent blue fill
            window.draw(rectangle);
        }
    }

    void drawBorder() {
        border.setSize(sf::Vector2f((maze_width - 2*border_size) *grid_size, (maze_height - 2*border_size) *grid_size));
        border.setPosition(border_size *grid_size, border_size *grid_size);
        border.setOutlineColor(border_color);
        border.setOutlineThickness(border_size *grid_size);  
        border.setFillColor(sf::Color::Transparent);

        border_print.setSize(sf::Vector2f((maze_width - 2*border_size), (maze_height - 2*border_size)));
        border_print.setPosition(border_size, border_size);
        border_print.setOutlineColor(border_color);
        border_print.setOutlineThickness(border_size);  
        border_print.setFillColor(sf::Color::Transparent);
    }

    void renderMazeToImage(const std::string& filename) {

        renderTexture.clear(sf::Color::White);

        drawObstacles(true);

        renderTexture.display(); 

        sf::Texture texture = renderTexture.getTexture();  // Get texture from render texture
        texture.copyToImage().saveToFile(filename);
    }

    Obstacle start, end, start_print, end_print;
    sf::RectangleShape border, border_print;
    sf::RenderWindow window;
    sf::RenderTexture renderTexture;
    int grid_size;
    int maze_width, maze_height;
    sf::Color grid_color;
    sf::Color obstacle_color;
    sf::Color border_color, startcolour, endcolour;
    int border_size;
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle> obstacles_print;
    std::string filename_maze = ecn::Maze::mazeFile("maze_generated_interactive.png");
    bool start_placed = false;

    sf::Vector2i first_click; // Position of the first click (starting point for the rectangle)
};

int main() {
    MazeEditor editor;
    editor.run();

    return 0;
}
