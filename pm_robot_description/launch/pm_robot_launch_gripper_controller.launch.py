import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # XYZ Axis Joint State Broadcaster
    spawn_pm_robot_parallel_gripper_JSB = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='pm_robot',
        arguments=[
            "pm_robot_parallel_gripper_JSB",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    # XYZ Axis Joint Trajectory Controller
    spawn_pm_robot_parallel_gripper_GAC = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_parallel_gripper_GAC",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_spawn_pm_robot_parallel_gripper_GAC= RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_pm_robot_parallel_gripper_JSB,
            on_exit=[spawn_pm_robot_parallel_gripper_GAC],
        )
    )

    # Define Launch Description

    ld = LaunchDescription()
    ld.add_action(spawn_pm_robot_parallel_gripper_JSB)
    ld.add_action(delay_spawn_pm_robot_parallel_gripper_GAC)
    return ld
