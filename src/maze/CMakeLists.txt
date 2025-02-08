cmake_minimum_required(VERSION 3.5)
project (maze)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SFML 2.5 REQUIRED graphics window system)

# Add a definition for the source directory
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_definitions(-DMAZES=\"${CMAKE_CURRENT_SOURCE_DIR}/mazes\")

include_directories(
        ${OpenCV_INCLUDE_DIRS} 
        include
)

# Library for maze functionality
add_library(maze_lib SHARED 
        include/maze.h 
        include/a_star.h 
        include/elastic_bands.h 
        include/point.h 
        include/position.h
        include/obstacle.h
        include/robot.h
        src/maze.cpp
        src/point.cpp
        src/elastic_bands.cpp
        src/robot.cpp
)

target_link_libraries(maze_lib ${OpenCV_LIBS} yaml-cpp)

# Node for `cmd_vel` Topic
add_executable(cmd_vel_node src/cmd_vel_node.cpp)
target_link_libraries(cmd_vel_node maze_lib ${OpenCV_LIBS} yaml-cpp)

ament_target_dependencies(maze_lib rclcpp geometry_msgs OpenCV yaml-cpp)
ament_target_dependencies(cmd_vel_node rclcpp geometry_msgs OpenCV yaml-cpp)

# ROS 2 Installation
install(TARGETS
    cmd_vel_node
    DESTINATION share/${PROJECT_NAME}/launch
    DESTINATION lib/${PROJECT_NAME}
)



# Export the package
ament_package()
