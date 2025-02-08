cmake_minimum_required(VERSION 3.5)
project (robonav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(yaml-cpp REQUIRED)
# find_package(SFML 2.5 REQUIRED graphics window system)

# Fix für doppelte sensor_msgs Include-Pfade (falls nötig)
# include_directories(${ament_INCLUDE_DIRS})

# Falls dein System nur mit dem doppelten sensor_msgs-Ordner funktioniert:
# sudo ln -s /opt/ros/humble/include/sensor_msgs/sensor_msgs /opt/ros/humble/include/sensor_msgs_fixed
# Dann folgende Zeile aktivieren:
# include_directories(/opt/ros/humble/include/sensor_msgs_fixed)

# Add definitions
# add_compile_definitions(SOURCE_DIR="${CMAKE_SOURCE_DIR}")
# add_compile_definitions(MAZES="${CMAKE_CURRENT_SOURCE_DIR}/mazes")

# include_directories(
#     ${OpenCV_INCLUDE_DIRS} 
#     include
# )

# # Library für Maze-Funktionalität
# add_library(maze_lib SHARED 
#     include/maze.h 
#     include/a_star.h 
#     include/elastic_bands.h 
#     include/point.h 
#     include/position.h
#     include/obstacle.h
#     include/robot.h
#     src/maze.cpp
#     src/point.cpp
#     src/elastic_bands.cpp
#     src/robot.cpp
# )

# target_link_libraries(maze_lib ${OpenCV_LIBS} yaml-cpp)
# ament_target_dependencies(maze_lib rclcpp geometry_msgs sensor_msgs OpenCV yaml-cpp)

# # Executable: joint_commands_node.cpp
# add_executable(joint_commands_node src/joint_commands_node.cpp)
# target_link_libraries(joint_commands_node maze_lib ${OpenCV_LIBS} yaml-cpp)
# ament_target_dependencies(joint_commands_node rclcpp geometry_msgs sensor_msgs OpenCV yaml-cpp)

# # Executable: main
# add_executable(main src/main.cpp)
# target_link_libraries(main maze_lib ${OpenCV_LIBS} yaml-cpp)
# ament_target_dependencies(main rclcpp geometry_msgs sensor_msgs OpenCV yaml-cpp)

# # ROS 2 Installation
# install(TARGETS
#     joint_commands_node
#     main
#     maze_lib  # Jetzt wird auch maze_lib installiert!
#     DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY description launch worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Export the package
ament_package()
