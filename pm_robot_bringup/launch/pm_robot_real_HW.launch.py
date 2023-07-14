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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
from yaml.loader import SafeLoader
import sys
from launch.substitutions import Command 

def generate_launch_description():

    bringup_config_path = os.path.join(get_package_share_directory('pm_robot_bringup'), 'config/pm_robot_bringup_config.yaml')
    
    f = open(bringup_config_path)
    bringup_config = yaml.load(f,Loader=SafeLoader)
    f.close()

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pm_robot_description'
    file_subpath = 'urdf/pm_robot_main.xacro'

    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    launch_moveit = False

    
    sim_time = False

    mappings={
        'launch_mode': 'real_HW',
        'with_Tool_MPG_10': str(bringup_config['pm_robot_tools']['MPG_10']['with_Tool_MPG_10']),
        'with_Gonio_Left': str(bringup_config['pm_robot_gonio_left']['with_Gonio_Left']),
        'with_Gonio_Right': str(bringup_config['pm_robot_gonio_right']['with_Gonio_Right']),
        'with_Tool_MPG_10_Jaw_3mm_Lens': str(bringup_config['pm_robot_tools']['MPG_10']['Config']['with_Tool_MPG_10_Jaw_3mm_Lens']),
        'with_Tool_SPT_Holder': str(bringup_config['pm_robot_tools']['SPT_Tool_Holder']['with_Tool_SPT_Holder']),
        'with_SPT_R_A1000_I500': str(bringup_config['pm_robot_tools']['SPT_Tool_Holder']['Config']['with_SPT_R_A1000_I500']),
    }

    robot_description_raw = xacro.process_file(pm_main_xacro_file, mappings=mappings).toxml()

    controler_param = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_sim_HW.yaml",
        ]
    )
    forward_command_action_server = Node(
            package="pm_robot_control_test",
            executable="forward_command_action_server",
            #name="pm_robot_IO_Axis_controller",
            parameters=[{'controller_param': controler_param}, 
                        {'robot_description': robot_description_raw}],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
    )

    moveit_config = (
        MoveItConfigsBuilder("pm_robot", package_name="pm_robot_moveit_config")
        .robot_description(file_path=pm_main_xacro_file,mappings=mappings)
        .robot_description_semantic(file_path="config/pm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        #"allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        #"capabilities": ParameterValue(
        #    LaunchConfiguration("capabilities"), value_type=str
        #),
        #"disable_capabilities": ParameterValue(
        #    LaunchConfiguration("disable_capabilities"), value_type=str
        #),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": "should_publish",
        #"publish_geometry_updates": should_publish,
        "publish_state_updates": "should_publish",
        #"publish_transforms_updates": should_publish,
        #"monitor_dynamics": False,
        "use_sim_time": sim_time,
    }    

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
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
            {"use_sim_time": sim_time},
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
                     'use_sim_time': sim_time}]  # add other parameters here if required
        #parameters=[moveit_config.robot_description],
    )

    robot_state_publisher_node_mov = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    robot_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_real_HW.yaml",
        ]
    )

    robot_description_command = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # control_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description_raw, robot_controllers_path],
    #     output="both",
    # )

    control_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description':robot_description_command},
                    robot_controllers_path],
        output="both",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[control_manager])

    launch_XYZT_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'pm_robot_launch_XYZT_controller_REAL.launch.py'
                ])
        ])#,
            #launch_arguments={                 # das muesste so funktionieren, noch nicht getestet; Ros2 control braucht aber use_sim_time nicht, da in yaml gegeben
            #    'use_sim_time': sim_time,
            #}.items()
        )
    
    launch_gonio_left_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'launch',
            'pm_robot_launch_gonio_left_controller.launch.py'
            ])
        ])
        )
    
    launch_gonio_right_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'launch',
            'pm_robot_launch_gonio_right_controller.launch.py'
            ])
        ])
        )
    
    launch_gonio_parallel_gripper_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'launch',
            'pm_robot_launch_gripper_controller.launch.py'
            ])
        ])
        )    
    
    # Define Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    #ld.add_action(control_manager)
    ld.add_action(delayed_controller_manager)
    if launch_moveit:
        ld.add_action(rviz_node)
        ld.add_action(run_move_group_node)
    ld.add_action(launch_XYZT_controllers)
    #ld.add_action(forward_command_action_server)
    # if (str(mappings['with_Gonio_Left']) == 'true'):
    #     ld.add_action(launch_gonio_left_controller)
    # if (str(mappings['with_Gonio_Right']) == 'true'):
    #     ld.add_action(launch_gonio_right_controller)
    # if (str(mappings['with_Tool_MPG_10']) == 'true'):
    #     ld.add_action(launch_gonio_parallel_gripper_controller)
    return ld
