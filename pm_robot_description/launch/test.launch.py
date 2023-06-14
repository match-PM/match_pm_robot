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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import sys

def generate_launch_description():


    moveit_config = (
        MoveItConfigsBuilder("pm_robot", package_name="pm_robot_moveit_config")
        .robot_description_semantic(file_path="config/pm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    launch_test = Node(
        package="test_node_1",
        executable="pm_moveit_tests",
        output="screen",
        parameters=[
                {
                    'robot_description_kinematics': moveit_config.robot_description_kinematics,
                }
            ],
    )

    ld = LaunchDescription()
    ld.add_action(launch_test)
       
    return ld
