import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
)
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
    bringup_config_path = os.path.join(get_package_share_directory("pm_robot_bringup"),"config/pm_robot_bringup_config.yaml",)

    with open(bringup_config_path) as f:
        bringup_config = yaml.safe_load(f)
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = "pm_robot_description"
    file_subpath = "urdf/pm_robot_main.xacro"

    # Use xacro to process the file
    pm_main_xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    launch_moveit = True
    sim_time = False

    mappings={
        'launch_mode': 'real_HW',
        'hardware': 'robot',
    }

    robot_description_raw = xacro.process_file(pm_main_xacro_file, mappings=mappings).toxml()

    controler_param = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_real_HW.yaml",
        ]
    )

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

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        # "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        # "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        # "capabilities": ParameterValue(
        #    LaunchConfiguration("capabilities"), value_type=str
        # ),
        # "disable_capabilities": ParameterValue(
        #    LaunchConfiguration("disable_capabilities"), value_type=str
        # ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": "should_publish",
        # "publish_geometry_updates": should_publish,
        # "publish_state_updates": "should_publish",
        # "publish_transforms_updates": should_publish,
        # "monitor_dynamics": False,
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
        parameters=move_group_params,    )

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
        emulate_tty=True
    )

    # Configure the node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_raw, "use_sim_time": sim_time}
        ],
        emulate_tty=True  
        # add other parameters here if required
        # parameters=[moveit_config.robot_description],
    )

    robot_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_real_HW.yaml",
        ]
    )

    robot_gonio_left_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_gonio_left_real_HW.yaml",
        ]
    )

    robot_gonio_right_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_gonio_right_real_HW.yaml",
        ]
    )

    robot_description_command = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    # control_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description_raw, robot_controllers_path],
    #     output="both",
    # )

    control_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_command},
            robot_controllers_path,
            robot_gonio_left_controllers_path,
            robot_gonio_right_controllers_path,
        ],
        output="both",
        emulate_tty=True
        # arguments=[                   # Set log level to debug
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
    )

        # Configure the node
    # this node listens to the states of the pneumatic and sets the joints
    pneumatic_controller_listener_node = Node(
        package="pneumatic_controller_listener",
        executable="pneumatic_controller_listener",
        name="pneumatic_controller_listener",
        output="both",
        parameters=[],
        emulate_tty=True  
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[control_manager])

    launch_XYZT_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "pm_robot_launch_XYZT_controller_REAL.launch.py",
                    ]
                )
            ]
        )  # ,
        # launch_arguments={                 # das muesste so funktionieren, noch nicht getestet; Ros2 control braucht aber use_sim_time nicht, da in yaml gegeben
        #    'use_sim_time': sim_time,
        # }.items()
    )

    pm_lights_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_lights_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        emulate_tty=True
    )

    pm_pneumatic_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_pneumatic_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        emulate_tty=True
    )

    pm_nozzle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_nozzle_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        emulate_tty=True
    )

    pm_uv_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_uv_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        emulate_tty=True
    )

    pm_sensor_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_sensor_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        emulate_tty=True
    )

    launch_gonio_left_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "pm_robot_launch_gonio_left_controller.launch.py",
                    ]
                )
            ]
        )
    )

    launch_gonio_right_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "pm_robot_launch_gonio_right_controller.launch.py",
                    ]
                )
            ]
        )
    )

    launch_gonio_parallel_gripper_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "pm_robot_launch_gripper_controller.launch.py",
                    ]
                )
            ]
        )
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

    gonio_orientation_solver_node = Node(
        package="gonio_orientation_solver",
        executable="gonio_orientation_solver",
        name="gonio_orientation_solver",
        # output="log",
        parameters=[
            {"use_sim_time": sim_time}
        ],
        emulate_tty=True,
    )

    primitive_skills_node = Node(
        package="pm_robot_primitive_skills",
        executable="pm_robot_primitive_skills",
        name="pm_robot_primitive_skills",
        # output="log",
        parameters=[
            {"use_sim_time": sim_time}
        ],
        emulate_tty=True,
    )

    delayed_rviz = TimerAction(period=15.0, actions=[rviz_node])
    delayed_move_group = TimerAction(period=15.0, actions=[run_move_group_node])
    delayed_pm_moveit_server = TimerAction(period=15.0, actions=[pm_moveit_server])
    delayed_pneumatic_controller_listener = TimerAction(period=15.0, actions=[pneumatic_controller_listener_node])

    # Define Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    # ld.add_action(control_manager)
    ld.add_action(delayed_controller_manager)
    ld.add_action(launch_XYZT_controllers)
    #ld.add_action(delayed_pneumatic_controller_listener)
    if launch_moveit:
        ld.add_action(delayed_rviz)
        ld.add_action(delayed_move_group)
        ld.add_action(delayed_pm_moveit_server)
        ld.add_action(gonio_orientation_solver_node)

    if bringup_config['pm_robot_gonio_left']['with_Gonio_Left']:
        ld.add_action(launch_gonio_left_controller)
    if bringup_config['pm_robot_gonio_right']['with_Gonio_Right']:
        ld.add_action(launch_gonio_right_controller)

    #ld.add_action(primitive_skills_node)
    ld.add_action(pm_lights_controller_spawner)
    ld.add_action(pm_pneumatic_controller_spawner)
    ld.add_action(pm_nozzle_controller_spawner)
    # ld.add_action(pm_uv_controller)
    ld.add_action(pm_sensor_controller)

    # if (str(mappings['with_Tool_MPG_10']) == 'true'):
    #     ld.add_action(launch_gonio_parallel_gripper_controller)
    return ld
