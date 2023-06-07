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
            "sim_mode:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("pm_robot_description"), "config", "pm_robot_control.yaml"]
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )

    robot_controller_forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
    )

    # rviz_config = PathJoinSubstitution(
    #     [FindPackageShare("pm_robot_description"), "rviz", "config.rviz"]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config],
    #     output="log",
    # )

    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    return LaunchDescription(
        [
            control_node,
            joint_state_broadcaster_spawner,
            robot_state_pub_node,
            robot_controller_spawner,
            #robot_controller_forward_spawner,
            # delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )
