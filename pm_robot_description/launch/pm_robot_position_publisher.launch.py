import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler, DeclareLaunchArgument
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

# launch file for pm_robot_forward_position_publisher to test forword_position_controller 

def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_positions_publisher.yaml",
        ]
    )

    controler_param = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_sim_HW.yaml",
        ]
    )

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pm_robot_description'
    file_subpath = 'urdf/pm_robot_main.xacro'


    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    pm_robot_configuration = {
                                'launch_mode':                    'sim_HW',              #real_HW sim_HW fake_HW real_sim_HW
                                'with_Tool_MPG_10':               'false',                  # Fix Needed !!!!!!!!!!!!!!
                                'with_Tool_MPG_10_Jaw_3mm_Lens':  'false',
                                'with_Gonio_Right':               'true',
                                'with_Gonio_Left':                'true',
                                'with_Tool_SPT_Holder':           'true',
                                'with_SPT_R_A1000_I500':          'true',
                              }

    mappings={
        'launch_mode': str(pm_robot_configuration['launch_mode']),
        'with_Tool_MPG_10': str(pm_robot_configuration['with_Tool_MPG_10']),
        'with_Gonio_Left': str(pm_robot_configuration['with_Gonio_Left']),
        'with_Gonio_Right': str(pm_robot_configuration['with_Gonio_Right']),
        'with_Tool_MPG_10_Jaw_3mm_Lens': str(pm_robot_configuration['with_Tool_MPG_10_Jaw_3mm_Lens']),
        'with_Tool_SPT_Holder': str(pm_robot_configuration['with_Tool_SPT_Holder']),
        'with_SPT_R_A1000_I500': str(pm_robot_configuration['with_SPT_R_A1000_I500']),
    }
        
    robot_description_raw = xacro.process_file(pm_main_xacro_file, mappings=mappings).toxml()


    return LaunchDescription(
        [
            Node(
                package="pm_robot_control_test",
                executable="pm_joint_trajectory_action_server",
                #name="pm_robot_IO_Axis_controller",
                parameters=[{'controller_param': controler_param}, 
                             {'robot_description': robot_description_raw}],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
            # Node(
            #     package="pm_robot_control_test",
            #     executable="pm_robot_joint_trajectory_publisher",
            #     name="publisher_joint_trajectory_controller",
            #     parameters=[position_goals],
            #     output={
            #         "stdout": "screen",
            #         "stderr": "screen",
            #     },
            # )
            # Node(
            #     package="pm_robot_control_test",
            #     executable="pm_robot_send_target_JTC",
            #     name="publisher_joint_trajectory_controller",
            #     parameters=[position_goals],
            #     output={
            #         "stdout": "screen",
            #         "stderr": "screen",
            #     },
            # )
        ]
    )

