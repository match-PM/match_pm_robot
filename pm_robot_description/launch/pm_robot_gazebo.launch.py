import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pm_robot_description'
    file_subpath = 'urdf/main.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "control.yaml",
        ]
    )

    # Configure the node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'pm_robot'],
                    output='screen')

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=['robot_description', robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments= ["joint_trajectory_controller"],
    )

    robot_controller_forward_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments= ["forward_position_controller"],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments= ["joint_state_broadcaster"],
    )


    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[robot_controller_forward_spawner],
        )
    )


    # Run the node
    return LaunchDescription([
        gazebo,
        control_node,
        robot_state_publisher_node,
        joint_broad_spawner,
        spawn_entity,
        #robot_controller_forward_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_controller,
    ])


