import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths
    sdf_file = "/home/ferdinand/model_editor_models/throbot/model.sdf"
    
    # Ensure the SDF file exists
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"SDF file not found: {sdf_file}")

    return LaunchDescription([
        # Declare the world file
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join('/opt/ros/humble/share/gazebo_ros', 'worlds', 'empty.world'),
            description='Gazebo world file'
        ),

        # Set robot_description parameter using URDF or SDF
        DeclareLaunchArgument(
            "robot_description",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robonav"), "urdf", "throbot.urdf"]
            ),
            description="Full path to robot URDF"
        ),

        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join('/opt/ros/humble/share/gazebo_ros/launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
        ),

        # Start the Controller Manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"use_sim_time": True, "update_rate": 50, "robot_description": LaunchConfiguration("robot_description")}
            ],
            output="screen"
        ),

        # Spawn the robot
        TimerAction(
            period=3.0,  # Wait for Gazebo to start
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'my_robot', '-file', sdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
                    output='screen'
                ),
            ]
        ),

        # Start the Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": LaunchConfiguration("robot_description")}]
        ),

        # Start the Joint Commands Publisher Node
        Node(
            package='maze',
            executable='joint_commands_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # Load and activate the controllers
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["diff_drive_controller"],
                    output="screen"
                ),
            ]
        ),
    ])
