import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Define paths to the robot description package and xacro file
    bot_description_path = get_package_share_directory('arm_bot_description')
    urdf_default_path = os.path.join(bot_description_path, 'urdf', 'project.urdf.xacro')
    rviz_config_path = os.path.join(bot_description_path, 'rviz', 'boturdf_config.rviz')
    controllers_yaml_path = os.path.join(bot_description_path, 'config', 'my_controllers.yaml')

    # Declare the launch argument for the URDF model
    urdf_path = DeclareLaunchArgument(
        name='model',
        default_value=urdf_default_path,
        description='Absolute path to robot urdf file'
    )

    # Process the URDF file using xacro
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]), value_type=str
        )
    }

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory('arm_bot_description'), 'worlds', 'test.world')}.items()
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
    )

    # Load joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Load diffbot_base_controller
    load_diffbot_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        urdf_path,
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
