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
        default_value='comp.world',  # Default world file
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

        # Include the Gazebo launch file with the custom world and GUI configuration

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
            # 'extra_gazebo_args': '--gui-config ' + os.path.join(
            #     get_package_share_directory(package_name), 'config', 'gazebo.ini'
            # ) + ' --verbose'  # Add verbose for debugging
        }.items()
    )

    cmd_vel_publisher = TimerAction(
        period=3.0,
        actions=[Node(
            package='robonav',
            executable='main_node',
            output='screen'
        )]
    )

    # Spawn robot at the given start position with orientation
    spawn_entity = TimerAction(
        period=4.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'robonav_robot',
                '-x', '-1.8',  # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE = - main_startposition
                '-y', '1.6',   # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 
                '-z', '0.0',   # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 
                '-R', '0.0',   # Roll angle in radians
                '-P', '0.0',   # Pitch angle in radians
                '-Y', '1.5708'    # Yaw angle in radians (orientation) # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE = +180
            ],
            output='screen'
        )]
    )

    # Start Rviz
    start_rviz = TimerAction(
        period=1.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([get_package_share_directory('robonav'), 'config', 'robonav.rviz'])],
            output='screen'
        )]
    )

    # # Start the map server
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     parameters=[{
    #         'yaml_filename': PathJoinSubstitution([
    #             get_package_share_directory('robonav'),
    #             'config',
    #             'map_server_params.yaml'  # Change this to the correct map file
    #         ])
    #     }],
    #     output='screen'
    # )

    # load_map = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'yaml_filename': '/home/ferdinand/2024-2025-Robotics-cup/src/robonav/worlds/robonav_map.yaml'
    #     }]
    # )

    # Launch them all!
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        spawn_entity,
        start_rviz,
        cmd_vel_publisher
    ])
