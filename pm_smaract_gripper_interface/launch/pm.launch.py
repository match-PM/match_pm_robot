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
                    FindPackageShare("pm_smaract_gripper_interface"),
                    "urdf",
                    "pm_smaract_gripper.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pm_smaract_gripper_interface"),
            "urdf",
            "pm_smaract_gripper_control.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    pm_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pm_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            control_node,
            pm_gripper_controller_spawner,
        ]
    )
