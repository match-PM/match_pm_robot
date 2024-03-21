import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # XYZ Axis Joint State Broadcaster
    Spawn_pm_robot_broadcaster = Node(
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
    Spawn_pm_robot_xyz_axis_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_xyz_axis_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # T Axis Joint Trajectory Controller
    Spawn_pm_robot_t_axis_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_t_axis_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    Spawn_pm_robot_io_axis_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_robot_IO_Axis_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    Spawn_pm_robot_pneumatic_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "pm_pneumatic_forward_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_Spawn_pm_robot_xyz_JTC = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_broadcaster,
            on_exit=[Spawn_pm_robot_xyz_axis_controller],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_Spawn_pm_robot_t_axis_JTC = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_broadcaster,
            on_exit=[Spawn_pm_robot_t_axis_controller],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_broadcaster,
            on_exit=[Spawn_pm_robot_io_axis_controller],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_spawn_pm_robot_pneumatic_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_pm_robot_broadcaster,
            on_exit=[Spawn_pm_robot_pneumatic_controller],
        )
    )

    # Define Launch Description

    ld = LaunchDescription()

    ld.add_action(Spawn_pm_robot_broadcaster)    
    ld.add_action(delay_Spawn_pm_robot_xyz_JTC)
    ld.add_action(delay_Spawn_pm_robot_t_axis_JTC)
    ld.add_action(delay_spawn_pm_robot_pneumatic_controller)
    #ld.add_action(delay_robot_controller_spawner_after_controller)

    return ld
