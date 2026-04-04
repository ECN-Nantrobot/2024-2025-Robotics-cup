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

    package_name = 'robonav'  # <--- CHANGE ME

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
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Include the rplidar_ros launch file
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'
        )])
    )

    # map_server_node = TimerAction(
    #     period=0.01,
    #     actions=[Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='map_server',
    #         output='screen',
    #         parameters=[{
    #             'yaml_filename': map_file,
    #             'use_sim_time': False,
    #             'always_send_full_map': True,
    #             'frame_id': 'map'
    #         }]
    #     )]
    # )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # amcl_node = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[amcl_param_file, {'use_sim_time': False}]
    # )

    # lifecycle_manager = TimerAction(
    #     period=2.0,  # !!enough for Gazebo + clock to tick so map doesnt start without a time
    #     actions=[Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_localization',
    #         output='screen',
    #         parameters=[{
    #             'use_sim_time': False,
    #             'autostart': True,
    #             # 'node_names': ['map_server', 'amcl']
    #             'node_names': ['map_server']
    #         }]
    #     )]
    # )

    main_node = TimerAction(
        period=0.1,
        actions=[Node(
            package='robonav',
            executable='main_node',
            output='screen'
        )]
    )

    laser_to_pointcloud_node = TimerAction(
        period=0.2,  # Add an 8-second delay
        actions=[Node(
            package='robonav',
            executable='laser_to_pointcloud_node',
            name='laser_to_pointcloud_node',
            output='screen',  # Output logs to the screen
            parameters=[
                {'use_sim_time': False} 
            ]
            # remappings=[
            #     ('scan', '/scan'),  # Remap the LaserScan topic if necessary
            #     ('pointcloud', '/pointcloud')  # Remap the PointCloud2 topic if necessary
            # ]
        )]
    )

    return LaunchDescription([
        rsp,
        # map_server_node,
        static_tf_node,
        # amcl_node,
        # lifecycle_manager,
        rplidar_launch,
        laser_to_pointcloud_node,
        main_node
    ])
