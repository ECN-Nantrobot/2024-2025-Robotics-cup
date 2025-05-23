from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('robonav')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    params = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        node_robot_state_publisher
    ])
