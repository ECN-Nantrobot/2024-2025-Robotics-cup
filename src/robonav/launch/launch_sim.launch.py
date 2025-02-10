import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction

def generate_launch_description():

    # Declare an argument for the custom Gazebo world
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',  # Default world file
        description='playmat.world'
    )

    package_name = 'robonav'  # <--- CHANGE ME

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': PathJoinSubstitution([
                get_package_share_directory(package_name),
                'worlds',
                LaunchConfiguration('world')
            ])
        }.items()
    )

    # Run the spawner node
    spawn_entity = TimerAction(
        period=6.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robonav_robot'],
            output='screen'
        )]
    )

    cmd_vel_publisher = TimerAction(
        period=7.0,
        actions=[Node(
            package='robonav',
            executable='main_node',
            output='screen'
        )]
    )


    # Launch them all!
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        spawn_entity,
        cmd_vel_publisher  
    ])
