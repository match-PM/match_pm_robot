import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    
    # Depricated
    # SmarPod Joint State Broadcaster
    Spawn_smarpod_JSB = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "smaract_hexapod_JSB",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    # SmarPod Joint Trajectory Controller
    Spawn_smarpod_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "smaract_hexapod_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Custom SmarPod Controller
    Spawn_custom_smarpod_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "smarpod_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Depricated
    # Delay start of smarpod_controller after `joint_state_broadcaster`
    delay_Spawn_smarpod_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_smarpod_JSB,
            on_exit=[Spawn_smarpod_controller],
        )
    )

    # Delay start of custom smarpod_controller after smaract_hexapod_controller
    delay_Spawn_custom_smarpod_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=Spawn_smarpod_controller,
            on_exit=[Spawn_custom_smarpod_controller],
        )
    )

    # Define Launch Description
    ld = LaunchDescription()
    #ld.add_action(Spawn_smarpod_JSB)
    #ld.add_action(delay_Spawn_smarpod_controller)
    ld.add_action(Spawn_smarpod_controller)
    ld.add_action(delay_Spawn_custom_smarpod_controller)
    return ld
