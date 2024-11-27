//Code by Jacek Wieczorek

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>

#include <fstream>  // Required for file operations
#include <maze.h>
#include <vector>  // Include for vector usage
#include <cstdlib> // Include for rand() and srand()

struct Node
{
        int x, y; //Node position - little waste of memory, but it allows faster generation
        void *parent; //Pointer to parent node
        char c; //Character to be displayed
        char dirs; //Directions that still haven't been explored
} ;

Node *nodes; //Nodes array
int width, height; //Maze dimensions

int init( )
{
        int i, j;
        Node *n;

        //Allocate memory for maze
//        nodes = static_cast<Node*>(calloc(width * height, sizeof(Node)));   // MODIFIED
        nodes = (Node *)(calloc(width * height, sizeof(Node)));   // MODIFIED
        if ( nodes == NULL ) return 1;

        //Setup crucial nodes
        for ( i = 0; i < width; i++ )
        {
                for ( j = 0; j < height; j++ )
                {
                        n = nodes + i + j * width;
                        if ( i * j % 2 )
                        {
                                n->x = i;
                                n->y = j;
                                n->dirs = 15; //Assume that all directions can be explored (4 youngest bits set)
                                n->c = ' ';
                        }
                        else n->c = '#'; //Add walls between nodes
                }
        }
        return 0;
}

Node *link( Node *n )
{
        //Connects node to random neighbor (if possible) and returns
        //address of next node that should be visited

        int x, y;
        char dir;
        Node *dest;

        //Nothing can be done if null pointer is given - return
        if ( n == NULL ) return NULL;

        //While there are directions still unexplored
        while ( n->dirs )
        {
                //Randomly pick one direction
                dir = ( 1 << ( rand( ) % 4 ) );

                //If it has already been explored - try again
                if ( ~n->dirs & dir ) continue;

                //Mark direction as explored
                n->dirs &= ~dir;

                //Depending on chosen direction
                switch ( dir )
                {
                        //Check if it's possible to go right
                        case 1:
                                if ( n->x + 2 < width )
                                {
                                        x = n->x + 2;
                                        y = n->y;
                                }
                                else continue;
                                break;

                        //Check if it's possible to go down
                        case 2:
                                if ( n->y + 2 < height )
                                {
                                        x = n->x;
                                        y = n->y + 2;
                                }
                                else continue;
                                break;

                        //Check if it's possible to go left
                        case 4:
                                if ( n->x - 2 >= 0 )
                                {
                                        x = n->x - 2;
                                        y = n->y;
                                }
                                else continue;
                                break;

                        //Check if it's possible to go up
                        case 8:
                                if ( n->y - 2 >= 0 )
                                {
                                        x = n->x;
                                        y = n->y - 2;
                                }
                                else continue;
                                break;
                }

                //Get destination node into pointer (makes things a tiny bit faster)
                dest = nodes + x + y * width;

                //Make sure that destination node is not a wall
                if ( dest->c == ' ' )
                {
                        //If destination is a linked node already - abort
                        if ( dest->parent != NULL ) continue;

                        //Otherwise, adopt node
                        dest->parent = n;

                        //Remove wall between nodes
                        nodes[n->x + ( x - n->x ) / 2 + ( n->y + ( y - n->y ) / 2 ) * width].c = ' ';

                        //Return address of the child node
                        return dest;
                }
        }

        //If nothing more can be done here - return parent's address
        /*return static_cast<Node*>(n->parent);    */      // MODIFIED
//        return n->parent;
        return (Node *)n->parent;
}

//void draw( )
//{
//        int i, j;

//        //Outputs maze to terminal - nothing special
//        for ( i = 0; i < height; i++ )
//        {
//                for ( j = 0; j < width; j++ )
//                {
//                        printf( "%c", nodes[j + i * width].c );
//                }
//                printf( "\n" );
//        }
//}

//void draw()
//{
//    std::string fullPath = std::string("/home/ecn/ecn_arpro/maze/mazes/test.txt");

//    std::ofstream outFile(fullPath);

//    if (!outFile.is_open())
//    {
//        fprintf(stderr, "Error: Could not open file for writing!\n");
//        exit(1);
//    }

//    int i, j;

//    // Outputs maze to the file
//    for ( i = 0; i < height; i++ )
//    {
//        for ( j = 0; j < width; j++ )
//        {
//            outFile << nodes[j + i * width].c;
//        }
//        outFile << "\n";  // Newline after each row
//    }

//    outFile.close();  // Close the file after writing
//}


//void draw(const char* filename)
//{
//    std::string fullPath = std::string("/home/ecn/ecn_arpro/maze/mazes/") + std::string(filename);

//    // Create an image with a black background (values of 255 for white)
//    cv::Mat image(height * 1, width* 1, CV_8UC3, cv::Scalar(255, 255, 255)); // 1x enlargement for better visibility

//    // Outputs maze to the image
//    for (int i = 0; i < height; i++)
//    {
//        for (int j = 0; j < width; j++)
//        {
//            if (nodes[j + i * width].c == ' ') // Assuming ' ' represents a path
//            {
//                // Draw a white square for the path
//                cv::rectangle(image,
//                              cv::Point(j * 1, i * 1),
//                              cv::Point((j + 1) * 1, (i + 1) * 1),
//                              cv::Scalar(0, 0, 0), // Color for the path (black)
//                              cv::FILLED);
//            }
//        }
//    }

//    // Save the image
//    if (!cv::imwrite(fullPath, image))
//    {
//        fprintf(stderr, "Error: Could not save image %s!\n", fullPath.c_str());
//        exit(1);
//    }

//    std::cout << "Maze saved as image: " << fullPath << std::endl;
//}


void draw(int percentage_to_remove)
{
    ecn::Maze mymaze(height, width);

    int wallCount = 0;
    std::vector<std::pair<int, int>> wallPositions;

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (nodes[j + i * width].c == ' ') // Assuming ' ' represents a path
            {
                mymaze.dig(j, i);
            }

            if (nodes[j + i * width].c == '#') // Assuming '#' represents a wall
            {
                wallCount++; // Count the walls
                wallPositions.push_back({j, i});
            }
        }
    }

    if(percentage_to_remove != 0)
    {
        int walls_to_remove = wallCount * percentage_to_remove/100;

        int i = 0;
        while (i < walls_to_remove)
        {
            int randomIndex = std::rand() % wallPositions.size();

            auto [x, y] = wallPositions[randomIndex];

            // Change the wall to a path if there is a wall
            //if(nodes[x + y * width].c == '#')
            if(!mymaze.isFree(x, y))
            {
                //nodes[x + y * width].c = ' ';
                mymaze.dig(x, y);
                i++;
            }
        }
    }

    mymaze.save();

    std::cout << "Maze saved as .png" << std::endl;
}




int main( int argc, char **argv )
{
        Node *start, *last;

        //Check argument count
        if ( argc < 4 )
        {
                fprintf( stderr, "%s: please specify maze dimensions and percentage of walls that should be randomly removed!\n", argv[0] );
                exit( 1 );
        }

        //Read maze dimensions from command line arguments
        if ( sscanf( argv[1], "%d", &width ) + sscanf( argv[2], "%d", &height ) < 2 )
        {
                fprintf( stderr, "%s: invalid maze size value!\n", argv[0] );
                exit( 1 );
        }

        //Allow only odd dimensions
        if ( !( width % 2 ) || !( height % 2 ) )
        {
                fprintf( stderr, "%s: dimensions must be odd!\n", argv[0] );
                exit( 1 );
        }

        //Do not allow negative dimensions
        if ( width <= 0 || height <= 0 )
        {
                fprintf( stderr, "%s: dimensions must be greater than 0!\n", argv[0] );
                exit( 1 );
        }

        //Seed random generator
        srand( time( NULL ) );

        //Initialize maze
        if ( init( ) )
        {
                fprintf( stderr, "%s: out of memory!\n", argv[0] );
                exit( 1 );
        }

        //Setup start node
        start = nodes + 1 + width;
        start->parent = start;
        last = start;

        int percentage_to_remove = atoi(argv[3]);

        //Connect nodes until start node is reached and can't be left
        while ( ( last = link( last ) ) != start );
        draw(percentage_to_remove);



        //show the image after generation
        std::string png_file = "/home/ecn/ecn_arpro/maze/mazes/maze.png";  // Adjust the path if needed
        cv::Mat maze_image = cv::imread(png_file);
        cv::imshow("Generated Maze", maze_image);  // Display the image in a window
        cv::waitKey(0);  // Wait for a key press before closing the window
}
