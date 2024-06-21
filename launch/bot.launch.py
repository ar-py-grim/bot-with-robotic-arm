import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # Define the path to the bot description package and xacro file
    bot_description_path = get_package_share_directory('arm_bot_description')
    urdf_path = os.path.join(bot_description_path, 'urdf', 'project.urdf.xacro')
    rviz_config_path = os.path.join(bot_description_path, 'rviz', 'boturdf_config.rviz')
    controllers_yaml_path = os.path.join(bot_description_path, 'config', 'my_controllers.yaml')

    # Process the xacro file to generate URDF
    doc = xacro.process_file(urdf_path)
    robot_description = {'robot_description': doc.toxml()}

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory('my_robot_bringup'), 'worlds', 'test_world.world')}.items()
    )

    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Define the node to spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # Define the controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml_path],
        output='screen',
        remappings=[
            ('/diffbot_base_controller/cmd_vel_unstamped', '/cmd_vel')
        ]
    )

    # Define the node to load joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Define the node to load diffbot_base_controller
    load_diffbot_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diffbot_base_controller'],
        output='screen'
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=urdf_path,
            description='Path to the URDF file'
        ),
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=rviz_config_path,
            description='Path to the RViz config file'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diffbot_base_controller],
            )
        ),
        rviz_node
    ])

