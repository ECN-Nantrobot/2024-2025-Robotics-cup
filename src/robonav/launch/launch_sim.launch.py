# # import os
# # from ament_index_python.packages import get_package_share_directory  # <- HINZUGEFÜGT!
# # from launch import LaunchDescription
# # from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
# # from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# # from launch.launch_description_sources import PythonLaunchDescriptionSource
# # from launch_ros.actions import Node
# # from launch.actions import ExecuteProcess  # Füge dies am Anfang hinzu
# # import xacro
# # from launch.substitutions import Command

# # def generate_launch_description():

# #     package_name = 'robonav'  # <--- CHANGE ME

# #     use_sim_time = LaunchConfiguration('use_sim_time', default='true')

# #     declare_use_sim_time = DeclareLaunchArgument(
# #         'use_sim_time',
# #         default_value='true',
# #         description='Use simulation (Gazebo) clock if true'
# #     )


# #     declare_world_file = DeclareLaunchArgument(
# #         'world',
# #         default_value='empty.world',
# #         description='Specify the world file to load'
# #     )


# #     # Include the Gazebo launch file with the custom world
# #     gazebo_launch = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource([os.path.join(
# #             get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
# #         )]),
# #         launch_arguments={
# #             'world': PathJoinSubstitution([
# #                 get_package_share_directory(package_name),
# #                 'worlds',
# #                 LaunchConfiguration('world')
# #             ])
# #         }.items()
# #     )


# #     # robot_description = Command([
# #     #     FindExecutable(name='xacro'), ' ', 
# #     #     PathJoinSubstitution('robonav'), 'urdf', 
# #     #     '/description/robot.urdf.xacro'
# #     # ])

# #     pkg_path = os.path.join(get_package_share_directory('robonav'))
# #     xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
# #     robot_description = xacro.process_file(xacro_file).toxml()
    

# #     robot_state_publisher_node = Node(
# #         package='robot_state_publisher',
# #         executable='robot_state_publisher',
# #         parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
# #         output='screen'
# #     )

# #     joint_state_publisher_node = Node(
# #         package='joint_state_publisher',
# #         executable='joint_state_publisher',
# #         output='screen',
# #     )

# #     robot_spawn_node = Node(
# #         package='gazebo_ros',
# #         executable='spawn_entity.py',
# #         arguments=['-topic', 'robot_description', '-entity', 'robot'],
# #         output='screen'
# #     )

# #     # load_joint_state_broadcaster = ExecuteProcess(
# #     #     cmd=['ros2', 'control', 'load_controller', 
# #     #          '--set-state', 'active', 'joint_state_broadcaster'],
# #     #     output='screen'
# #     # )

# #     load_diff_drive_controller = ExecuteProcess(
# #         cmd=['ros2', 'control', 'load_controller',
# #                 '--set-state', 'active', 'diff_drive_controller'],
# #         output='screen'
# #     )

# #     controller_manager_node = Node(
# #         package="controller_manager",
# #         executable="ros2_control_node",
# #         parameters=[{
# #             "robot_description": robot_description,
# #             "use_sim_time": use_sim_time
# #         },
# #         PathJoinSubstitution([
# #             get_package_share_directory("robonav"),
# #             "config",
# #             "diff_drive_controller.yaml"
# #         ])],
# #         output="screen"
# #     )



# #     # run_plotjuggler = ExecuteProcess(
# #     #     cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
# #     #     output='screen',
# #     #     shell=True
# #     # )

# #     return LaunchDescription([
# #         declare_use_sim_time,
# #         declare_world_file,
# #         gazebo_launch,
# #         robot_state_publisher_node,
# #         controller_manager_node,  # Füge dies hier hinzu
# #         robot_spawn_node,
# #         joint_state_publisher_node,
# #         # load_joint_state_broadcaster,
# #         load_diff_drive_controller,
# #         # run_plotjuggler
# #     ])







# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import Command
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import LogInfo
# import os
# from launch.conditions import IfCondition
# from launch.substitutions import PythonExpression


# def generate_launch_description():
#     # Pfad zur Roboterbeschreibung (URDF/Xacro)
#     robot_description_path = PathJoinSubstitution([
#         FindPackageShare("robonav"),  # Name deines ROS2-Pakets
#         "description",  # Ordner mit URDF/Xacro-Dateien
#         "robot.urdf"
#     ])

#     # Gazebo-Launch-File
#     gazebo_launch_path = PathJoinSubstitution([
#         FindPackageShare("gazebo_ros"),  # Gazebo-Plugin-Paket
#         "launch",
#         "gazebo.launch.py"
#     ])

#     # Argumente für die Launch-Datei
#     declare_use_sim_time = DeclareLaunchArgument(
#         name="use_sim_time", default_value="true", description="Use simulation time"
#     )

#     declare_robot_state_publisher = DeclareLaunchArgument(
#         name="robot_description_file", default_value=robot_description_path,
#         description="Path to robot URDF/Xacro file"
#     )

#     # Gazebo starten
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(gazebo_launch_path),
#         launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items()
#     )

#     # Spawn den Roboter in Gazebo
#     spawn_robot = Node(
#         package="gazebo_ros",
#         executable="spawn_entity.py",
#         arguments=[
#             "-entity", "my_robot",
#             "-topic", "/robot_description",
#         ],
#         output="screen"
#     )

#     # Node für den Robot State Publisher
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="screen",
#         parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
#         arguments=[LaunchConfiguration("robot_description_file")]
#     )



#     ros2_control_params=[
#         PathJoinSubstitution([
#             FindPackageShare("robonav"),  # Paketname (muss exakt stimmen!)
#             "config",
#             "ros2_control_params.yaml"
#         ])
#     ]



#     controller_manager = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[
#             {"robot_description": Command(["xacro ", LaunchConfiguration("robot_description_file")])},
#             ros2_control_params # Datei mit den Controller-Parametern
#         ],
#         output="screen"
#     )


#     diff_drive_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["DiffDriveController", "-c", "/controller_manager"],
#         output="screen"
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
#         output="screen"
#     )



#     # # Controller spawnen
#     # controller_spawner = Node(
#     #     package="controller_manager",
#     #     executable="spawner",
#     #     arguments=["DiffDriveController", "-c", "/controller_manager"],
#     #     output="screen"
#     # )

#     return LaunchDescription([
#         declare_use_sim_time,
#         declare_robot_state_publisher,
#         gazebo,
#         robot_state_publisher,
#         spawn_robot,
#         # controller_spawner,
#         controller_manager,
#         diff_drive_controller_spawner,
#         joint_state_broadcaster_spawner
#     ])


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    package_name = 'robonav'  # Replace with your package name

    # 1. URDF/Xacro Description
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # 2. Gazebo Launch
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    ])

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',  # Or your world file
        description='Gazebo world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]  # Use sim time!
    )

    # 4. Spawn Entity (Robot in Gazebo)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', '/robot_description'],  # Use /robot_description
        output='screen'
    )

    # 5. Controller Manager
    controller_manager_params = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "diff_drive_controller.yaml"  # Path to your controller config file
    ])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},  # Use sim time!
            controller_manager_params # Your controller parameters
        ],
        output="screen"
    )

    # 6. Controller Spawner (Diff Drive)
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # 7. Controller Spawner (Joint State Broadcaster)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        world_file_arg,  # Declare the world file argument
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner,
    ])



































#     # use_sim_time = LaunchConfiguration('use_sim_time')
#     # use_sim_time_arg = DeclareLaunchArgument(
#     #     'use_sim_time',
#     #     default_value='true',
#     #     description='Use simulation (Gazebo) clock if true'
#     # )




#     # # Include the robot_state_publisher launch file
#     # # rsp = IncludeLaunchDescription(
#     # #     PythonLaunchDescriptionSource([os.path.join(
#     # #         get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
#     # #     )]),
#     # #     launch_arguments={'use_sim_time': 'true'}.items()
#     # # )

 
#     # pkg_path = os.path.join(get_package_share_directory('robonav'))
#     # xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
#     # robot_description_config = xacro.process_file(xacro_file)
#     # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    
#     # node_robot_state_publisher = Node(
#     #     package='robot_state_publisher',
#     #     executable='robot_state_publisher',
#     #     output='screen',
#     #     parameters=[params]
#     # )

#     # # Run the spawner node
#     # spawn_entity = Node(
#     #     package='gazebo_ros',
#     #     executable='spawn_entity.py',
#     #     arguments=['-topic', 'robot_description', '-entity', 'robonav_robot'],
#     #     output='screen'
#     # )

#     # # Start Controller Manager
#     # controller_manager = Node(
#     #     package="controller_manager",
#     #     executable="ros2_control_node",
#     #     parameters=[get_package_share_directory('robonav') + '/config/diff_drive_controller.yaml'],
#     #     output="screen"
#     # )

#     #     # Lade den `diff_drive_controller` nach 5 Sekunden
#     # start_diff_drive_controller = TimerAction(
#     #     period=3.0,
#     #     actions=[
#     #         Node(
#     #             package="controller_manager",
#     #             executable="spawner",
#     #             arguments=["diff_drive_controller"],
#     #             output="screen"
#     #         ),
#     #     ]
#     # )

#     # # Launch them all!
#     # return LaunchDescription([
#     #     use_sim_time_arg,
#     #     world_file_arg,
#     #     # rsp,
#     #     gazebo,
#     #     node_robot_state_publisher,
#     #     spawn_entity,
#     #     controller_manager,
#     #     # joint_commands,
#     #     start_diff_drive_controller
#     # ])






# # import os
# # from ament_index_python.packages import get_package_share_directory  # <- HINZUGEFÜGT!
# # from launch import LaunchDescription
# # from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
# # from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# # from launch.launch_description_sources import PythonLaunchDescriptionSource
# # from launch_ros.actions import Node
# # from launch_ros.substitutions import FindPackageShare


# # def generate_launch_description():

# #     package_name = 'robonav'  # Dein ROS 2 Paketname
# #     world_file = 'empty.world'  # Dein Gazebo-Welt-File
# #     sdf_file = PathJoinSubstitution([FindPackageShare(package_name), "description", "robot.sdf"])

# #     # Argument für die Welt
# #     world_file_arg = DeclareLaunchArgument(
# #         'world',
# #         default_value=world_file,
# #         description='Gazebo world file'
# #     )

# #     # Include robot_state_publisher Launch-File
# #     rsp = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource([os.path.join(
# #             get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
# #         )]),
# #         launch_arguments={'use_sim_time': 'true'}.items()
# #     )

# #     # Include Gazebo mit der Welt
# #     gazebo = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource([os.path.join(
# #             get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
# #         )]),
# #         launch_arguments={
# #             'world': PathJoinSubstitution([
# #                 get_package_share_directory(package_name),
# #                 'worlds',
# #                 LaunchConfiguration('world')
# #             ])
# #         }.items()
# #     )

# #     # Start Controller Manager
# #     controller_manager = Node(
# #         package="controller_manager",
# #         executable="ros2_control_node",
# #         parameters=[{"use_sim_time": True, "update_rate": 50}],
# #         output="screen"
# #     )

# #     # Spawne den Roboter in Gazebo (nach 3 Sekunden, damit Gazebo bereit ist)
# #     spawn_robot = TimerAction(
# #         period=3.0,
# #         actions=[
# #             Node(
# #                 package='gazebo_ros',
# #                 executable='spawn_entity.py',
# #                 arguments=['-entity', 'robonav_robot', '-file', sdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
# #                 output='screen'
# #             ),
# #         ]
# #     )

# #     # Start `joint_commands_node` (zum Senden von Steuerbefehlen)
# #     joint_commands = Node(
# #         package='robonav',
# #         executable='joint_commands_node',
# #         output='screen',
# #         parameters=[{'use_sim_time': True}],
# #         arguments=['--ros-args', '--log-level', 'info']
# #     )

# #     # Lade den `diff_drive_controller` nach 5 Sekunden
# #     start_diff_drive_controller = TimerAction(
# #         period=5.0,
# #         actions=[
# #             Node(
# #                 package="controller_manager",
# #                 executable="spawner",
# #                 arguments=["diff_drive_controller"],
# #                 output="screen"
# #             ),
# #         ]
# #     )

# #     # Alles zusammen starten
# #     return LaunchDescription([
# #         world_file_arg,
# #         rsp,
# #         gazebo,
# #         controller_manager,
# #         spawn_robot,
# #         joint_commands,
# #         start_diff_drive_controller,
# #     ])
