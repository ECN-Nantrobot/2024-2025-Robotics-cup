cmake_minimum_required(VERSION 3.5)
project (maze)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# Add a definition for the source directory
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")

set(OpenCV_DIR "C:/Users/ferdi/Libraries/opencv/build")
find_package(OpenCV REQUIRED)

set(yaml-cpp_DIR "C:/Users/ferdi/Libraries/yaml-cpp/install/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

find_package(SFML 2.5 REQUIRED graphics window system)

add_definitions(-DMAZES=\"${CMAKE_CURRENT_SOURCE_DIR}/mazes\")

include_directories(${YamlCpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

add_library(maze SHARED 
        include/maze.h 
        include/a_star.h 
        include/elastic_bands.h 
        include/point.h 
        include/position.h
        include/obstacle.h
        src/maze.cpp
        src/point.cpp
        src/maze.cpp
        src/elastic_bands.cpp)
target_link_libraries(maze ${OpenCV_LIBS})


# Generator-Executable
add_executable(generator src/maze_generation.cpp)
target_link_libraries(generator maze)

# main-Executable
add_executable(solve src/main.cpp)
target_link_libraries(solve maze)

# Interactive Generation-Executable
add_executable(interactive_generation src/interactive_generation.cpp)
target_link_libraries(interactive_generation sfml-graphics sfml-window sfml-system maze ${OpenCV_LIBS})

