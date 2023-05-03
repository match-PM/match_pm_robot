import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pm_robot_description'
    file_subpath = 'urdf/pm_robot_main.xacro'

    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(
        get_package_share_directory(pkg_name), file_subpath)

    pm_robot_configuration = {'with_Tool_MPG_10':               'false',
                              'with_Tool_MPG_10_Jaw_3mm_Lens':  'false',
                              'with_Gonio_Right':               'true',
                              'with_Gonio_Left':                'true',
                              'with_Tool_SPT_Holder':           'false',
                              'with_SPT_R_A1000_I500':          'false',
                              }

    robot_description_raw = xacro.process_file(pm_main_xacro_file,
                                               mappings={
                                                   'with_Tool_MPG_10': str(pm_robot_configuration['with_Tool_MPG_10']),
                                                   'with_Gonio_Left': str(pm_robot_configuration['with_Gonio_Left']),
                                                   'with_Gonio_Right': str(pm_robot_configuration['with_Gonio_Right']),
                                                   'with_Tool_MPG_10_Jaw_3mm_Lens': str(pm_robot_configuration['with_Tool_MPG_10_Jaw_3mm_Lens']),
                                                   'with_Tool_SPT_Holder': str(pm_robot_configuration['with_Tool_SPT_Holder']),
                                                   'with_SPT_R_A1000_I500': str(pm_robot_configuration['with_SPT_R_A1000_I500']),
                                               }).toxml()


    moveit_config = (
        MoveItConfigsBuilder("pm_robot", package_name="pm_robot_moveit_config")
        #.robot_description(robot_description_raw) not working
        .robot_description(file_path="config/pm_robot_link.urdf.xacro")
        #.robot_description(file_path="config/pm_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/pm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("pm_robot_moveit_config"), "config"
    )

    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


    # Configure the node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='both',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]  # add other parameters here if required
        #parameters=[moveit_config.robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'pm_robot'],
                        output='screen')

    robot_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control.yaml",
        ]
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("pm_robot_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_raw, robot_controllers_path],
        output="both",
    )


    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #arguments=["joint_trajectory_controller"], previous
        arguments=[
            "pm_robot_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_forward_spawner = Node(
        package='controller_manager',
        executable='spawner',
        #arguments=["forward_position_controller"],
        arguments=[
            "pm_robot_forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='pm_robot',
        #arguments=["joint_state_broadcaster"], previous
        arguments=[
            "pm_robot_joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
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
        
        rviz_node,
        gazebo,
        robot_state_publisher_node,
        run_move_group_node,
        control_node,
        joint_broad_spawner,
        spawn_entity,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_controller,

    ])
