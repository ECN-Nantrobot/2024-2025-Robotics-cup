cmake_minimum_required(VERSION 3.5)
project(robonav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Library für Maze-Funktionalität
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

add_library(serial_lib STATIC
    src/serial/src/serial.cc
    src/serial/src/impl/unix.cc
)

target_include_directories(serial_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/serial/include
)

target_link_libraries(maze_lib ${OpenCV_LIBS} yaml-cpp)
ament_target_dependencies(maze_lib rclcpp geometry_msgs OpenCV yaml-cpp)

add_compile_definitions(SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_compile_definitions(MAPS="${CMAKE_CURRENT_SOURCE_DIR}/maps")

add_executable(main_node src/main.cpp)
target_link_libraries(main_node maze_lib ${OpenCV_LIBS} yaml-cpp serial_lib ${cv_bridge_LIBRARIES})
ament_target_dependencies(main_node rclcpp geometry_msgs OpenCV yaml-cpp tf2 tf2_ros nav_msgs cv_bridge)

target_include_directories(maze_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(main_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  main_node
  maze_lib  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds include description config scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
