import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction
import math
from launch.actions import ExecuteProcess

def generate_launch_description():

    package_name = 'robonav'

    map_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'maps',
        'beacon_map.yaml'
    ])

    amcl_param_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'config',
        'amcl.yaml'
    ])

    # Ensure the files exist
    assert os.path.exists(map_file.perform({})), f"Map file not found: {map_file.perform({})}"
    assert os.path.exists(amcl_param_file.perform({})), f"AMCL parameter file not found: {amcl_param_file.perform({})}"


    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 
            'launch', 
            'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file with the custom world and GUI configuration
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'competition.world'
            ),
            'gui': 'false' ###
        }.items()
    )
    

    cmd_vel_publisher = TimerAction(
        period=6.0,
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

    # Spawn robot at the given start position with orientation
    spawn_entity = TimerAction(
        period=0.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'robonav_robot',
                '-x', str(x_start),  
                '-y', str(y_start), 
                # '-z', '0.0', 
                # '-R', '0.0',  
                # '-P', '0.0',  
                '-Y', str(theta)    # Yaw angle in radians (orientation) # -180 degrees
            ],
            output='screen'
        )]
    )

    # Start Rviz
    start_rviz = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([get_package_share_directory('robonav'), 'config', 'robonav.rviz'])],
            output='screen'
        )]
    )

    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     # arguments=['0.12', '0.0', '0.17', '0', '0', '0', 'odom', 'lidar'],
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    #     output='screen'
    # )


    map_server_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': True,
                'always_send_full_map': True,
                'frame_id': 'map'
            }]
        )]
    )


    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_param_file, {'use_sim_time': True}]
    )

    



    laser_to_pointcloud_node = Node(
        package='robonav',
        executable='laser_to_pointcloud_node',
        name='laser_to_pointcloud_node',
        output='screen',  # Output logs to the screen
        parameters=[
            {'use_sim_time': True} 
        ],
        remappings=[
            ('scan', '/scan'),  # Remap the LaserScan topic if necessary
            ('pointcloud', '/pointcloud')  # Remap the PointCloud2 topic if necessary
        ]
    )



    lifecycle_manager = TimerAction(
        period=5.0,  # !!enough for Gazebo + clock to tick so map doesnt start without a time
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )]
    )




    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        start_rviz,
        cmd_vel_publisher,
        # static_transform_publisher,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        laser_to_pointcloud_node
        # initial_pose_timer
    ])
