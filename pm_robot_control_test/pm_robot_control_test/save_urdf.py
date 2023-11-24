import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

import xacro

class SaveURDF(Node):

    def __init__(self):
        super().__init__('save_urdf')


        self.get_logger().info(f'URDF init')
        self.save_urdf_file()

    def save_urdf_file(self):

        pkg_path = os.path.join(get_package_share_directory('pm_robot_description'))
        xacro_file = os.path.join(pkg_path,'urdf','pm_robot_main.xacro')
        robot_description = xacro.process_file(xacro_file)
        formatted_xml = robot_description.toprettyxml(indent='  ')

        urdf_file = '/home/match-mover/Documents/robot.urdf'

        with open(urdf_file, 'w') as file:
            file.write(formatted_xml)

        # # Open the file in write mode
        # with open(file_path, "w") as file:
        #     # Write the formatted XML content to the file
        #     file.write(formatted_xml)

        self.get_logger().info(f'URDF "{file}"')
        return

          
    
def main(args=None):
    rclpy.init(args=args)

    save_urdf = SaveURDF()

    rclpy.spin(save_urdf)
    save_urdf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

