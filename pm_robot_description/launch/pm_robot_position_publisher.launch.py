from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# launch file for pm_robot_forward_position_publisher to test forword_position_controller 

def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("pm_robot_description"),
            "config",
            "pm_robot_positions_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="pm_robot_control_test",
                executable="pm_robot_forward_position_publisher",
                name="publisher_forward_position_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )
