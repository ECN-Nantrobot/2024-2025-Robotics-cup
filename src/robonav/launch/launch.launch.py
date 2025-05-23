import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction
import math

def generate_launch_description():

    # Declare an argument for the custom Gazebo world
    # world_file_arg = DeclareLaunchArgument(
    #     'world',
    #     default_value='draft.world',  # Default world file
    #     description='empty.world'
    # )

    package_name = 'robonav'  # <--- CHANGE ME

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # # Include the Gazebo launch file with the custom world and GUI configuration
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    #     )]),
    #     launch_arguments={
    #         'world': PathJoinSubstitution([
    #             get_package_share_directory(package_name),
    #             'worlds',
    #             LaunchConfiguration('world')
    #         ]),
    #         'gui': 'false'
    #     }.items()
    # )

    cmd_vel_publisher = TimerAction(
        period=3.0,
        actions=[Node(
            package='robonav',
            executable='main_node',
            output='screen'
        )]
    )

    x_start_cv_frame = 270 # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 
    y_start_cv_frame = 150 # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE 
    theta_start_cv_frame = -90 # <--- CHANGE MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

    x_start = x_start_cv_frame/100
    y_start = 2 - y_start_cv_frame/100
    theta = math.radians(-theta_start_cv_frame)

    # # Spawn robot at the given start position with orientation
    # spawn_entity = TimerAction(
    #     period=4.0,
    #     actions=[Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         arguments=[
    #             '-topic', 'robot_description',
    #             '-entity', 'robonav_robot',
    #             '-x', str(x_start),  
    #             '-y', str(y_start), 
    #             # '-z', '0.0', 
    #             # '-R', '0.0',  
    #             # '-P', '0.0',  
    #             '-Y', str(theta)    # Yaw angle in radians (orientation) # -180 degrees
    #         ],
    #         output='screen'
    #     )]
    # )

    # # Start Rviz
    # start_rviz = TimerAction(
    #     period=1.0,
    #     actions=[Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', PathJoinSubstitution([get_package_share_directory('robonav'), 'config', 'robonav.rviz'])],
    #         output='screen'
    #     )]
    # )

    # # Start the map server
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     parameters=[{
    #         'yaml_filename': PathJoinSubstitution([
    #             get_package_share_directory('robonav'),
    #             'config',
    #             'map_server_params.yaml'
    #         ])
    #     }],
    #     output='screen'
    # )

    # image_publisher = Node(
    #     package='robonav',  # <--- Ersetze durch dein Paket
    #     executable='image_publisher.py',  # Name der ausführbaren Datei
    #     name='image_publisher',
    #     output='screen'
    # )

    # static_tf_pub = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    # )


    # Launch them all!
    return LaunchDescription([
        # world_file_arg,
        rsp,
        # gazebo,
        # spawn_entity,
        # start_rviz,
        cmd_vel_publisher,
        # map_server,
        # image_publisher  
        # static_tf_pub
    ])
