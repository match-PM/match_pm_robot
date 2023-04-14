import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def generate_launch_description():
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('pm_robot_description'))
    xacro_file = os.path.join(pkg_path,'urdf','pm_robot_main.xacro')
    #robot_description = xacro.process_file(xacro_file)
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rviz = get_package_share_directory('rviz2')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    model_arg = DeclareLaunchArgument(name='pm_robot', default_value=str(xacro_file),
                                      description='Absolute path to robot urdf file')
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(pkg_rviz),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('pm_robot')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments= ["forward_position_controller"],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments= ["joint_state_broadcaster"],
    )


    return LaunchDescription([
        gui_arg,
        model_arg,

        rviz_arg,
        #joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        #joint_broad_spawner,
        #robot_controller_spawner,
    ])
