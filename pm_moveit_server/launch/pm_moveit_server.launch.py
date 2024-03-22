import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from yaml.loader import SafeLoader
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import sys


def generate_launch_description():
    bringup_config_path = os.path.join(
        get_package_share_directory("pm_robot_bringup"),
        "config/pm_robot_bringup_config.yaml",
    )

    f = open(bringup_config_path)
    bringup_config = yaml.load(f, Loader=SafeLoader)
    f.close()

    # Specify the name of the package and path to xacro file within the package
    file_subpath = "urdf/pm_robot_main.xacro"

    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(
        get_package_share_directory("pm_robot_description"), file_subpath
    )

    sim_time = True

    if sim_time:
        mappings = {
            "launch_mode": "sim_HW",
            "with_Gonio_Left": str(
                bringup_config["pm_robot_gonio_left"]["with_Gonio_Left"]
            ),
            "with_Gonio_Right": str(
                bringup_config["pm_robot_gonio_right"]["with_Gonio_Right"]
            ),
        }
    else:
        mappings = {
            "launch_mode": "real_HW",
            "with_Gonio_Left": str(
                bringup_config["pm_robot_gonio_left"]["with_Gonio_Left"]
            ),
            "with_Gonio_Right": str(
                bringup_config["pm_robot_gonio_right"]["with_Gonio_Right"]
            ),
        }

    moveit_config = (
        MoveItConfigsBuilder("pm_robot", package_name="pm_robot_moveit_config")
        .robot_description(file_path=pm_main_xacro_file, mappings=mappings)
        .robot_description_semantic(file_path="config/pm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    pm_moveit_server = Node(
        package="pm_moveit_server",
        executable="pm_moveit_server",
        name="pm_moveit_server",
        # output="log",
        parameters=[
            {"use_sim_time": sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        emulate_tty=True,
    )

    # Define Launch Description
    ld = LaunchDescription()

    ld.add_action(pm_moveit_server)
    return ld
