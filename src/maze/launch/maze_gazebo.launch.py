# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Paths to your world and robot model
#     sdf_file = "/home/ferdinand/model_editor_models/throbot/model.sdf"
#     world_file = "/home/ferdinand/eurobot25_playmat/worlds/plalymat.world"

#     # Ensure the files exist before launching
#     if not os.path.exists(world_file):
#         raise FileNotFoundError(f"World file not found: {world_file}")
#     if not os.path.exists(sdf_file):
#         raise FileNotFoundError(f"SDF file not found: {sdf_file}")

#     return LaunchDescription([
#         # Set Gazebo environment variables
#         DeclareLaunchArgument(
#             'world', default_value=world_file,
#             description='Path to the Gazebo world file'
#         ),

#         # Launch Gazebo with the correct world
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join('/opt/ros/humble/share/gazebo_ros/launch', 'gazebo.launch.py')
#             ),
#             launch_arguments={'world': world_file}.items(),
#         ),

#         # Spawn the robot in Gazebo
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=['-entity', 'my_robot', '-file', sdf_file, '-x', '0', '-y', '0', '-z', '0.5'],
#             output='screen'
#         ),
#     ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your robot SDF model
    sdf_file = "/home/ferdinand/model_editor_models/throbot/model.sdf"

    # Ensure the file exists before launching
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"SDF file not found: {sdf_file}")

    return LaunchDescription([
        # Declare Gazebo world argument (you can customize later)
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join('/opt/ros/humble/share/gazebo_ros', 'worlds', 'empty.world'),
            description='Gazebo world file'
        ),

        # Start Gazebo with ROS2 plugins
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join('/opt/ros/humble/share/gazebo_ros/launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
        ),

        # Delay spawning so Gazebo loads first
        TimerAction(
            period=5.0,  # Wait 5 seconds before spawning
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'my_robot', '-file', sdf_file, '-x', '0', '-y', '0', '-z', '0.5'],
                    output='screen'
                ),
            ]
        )
    ])

