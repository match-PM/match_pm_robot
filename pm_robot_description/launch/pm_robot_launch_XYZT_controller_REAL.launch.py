import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # XYZ Axis Joint State Broadcaster
    Spawn_pm_robot_JSB = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='pm_robot',
        arguments=[
            "pm_robot_JSB",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # XYZ Axis Joint Trajectory Controller
    Spawn_pm_robot_xyz_axis_FPC = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_xyz_axis_FPC",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # T Axis Joint Trajectory Controller
    Spawn_pm_robot_t_axis_FPC = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_xyz_axis_FPC",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    Spawn_pm_robot_io_axis_FPC = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_IO_Axis_FPC",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_Spawn_pm_robot_xyz_FPC = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_JSB,
            on_exit=[Spawn_pm_robot_xyz_axis_FPC],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_Spawn_pm_robot_t_axis_FPC = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_JSB,
            on_exit=[Spawn_pm_robot_t_axis_FPC],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_JSB,
            on_exit=[Spawn_pm_robot_io_axis_FPC],
        )
    )

    # Define Launch Description

    ld = LaunchDescription()

    ld.add_action(Spawn_pm_robot_JSB)    
    ld.add_action(delay_Spawn_pm_robot_xyz_FPC)
    # ld.add_action(delay_Spawn_pm_robot_t_axis_FPC)
    # ld.add_action(delay_robot_controller_spawner_after_controller)

    return ld
