import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # XYZ Axis Joint State Broadcaster
    Spawn_pm_robot_gonio_left_JSB = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='pm_robot',
        arguments=[
            "pm_robot_gonio_left_JSB",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    # XYZ Axis Joint Trajectory Controller
    Spawn_pm_robot_gonio_left_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_gonio_left_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_Spawn_pm_robot_gonio_left_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_gonio_left_JSB,
            on_exit=[Spawn_pm_robot_gonio_left_controller],
        )
    )

    # Define Launch Description

    ld = LaunchDescription()
    ld.add_action(Spawn_pm_robot_gonio_left_JSB)
    ld.add_action(delay_Spawn_pm_robot_gonio_left_controller)
    return ld
