from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_description = """
    <robot name="test_robot">
        <link name="base_link"/>

        <ros2_control name="test_controller" type="system">  <hardware>
                <plugin>gazebo_ros2_control/GazeboSystem</plugin>
            </hardware>
        </ros2_control>

        <gazebo>
            <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
                <robotSimType>gazebo_ros2_control/GazeboSystem</robotSimType>
            </plugin>
        </gazebo>
    </robot>
    """

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription([controller_manager])