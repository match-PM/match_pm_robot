from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pm_robot_description"),
                    "urdf",
                    "pm_robot_main.xacro",
                ]
            ),
            " ",
            "launch_mode:=real_HW",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_control_real_HW.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    pm_lights_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_lights_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    pm_pneumatic_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_pneumatic_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    pm_nozzle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_nozzle_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    pm_robot_xyz_axis_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_robot_xyz_axis_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            control_node,
            pm_lights_controller_spawner,
            pm_robot_xyz_axis_controller,
            pm_pneumatic_controller_spawner,
            pm_nozzle_controller_spawner,
            robot_state_pub_node,
        ]
    )
