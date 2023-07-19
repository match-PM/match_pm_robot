import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory

import yaml
from yaml.loader import SafeLoader

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from pm_robot_interfaces.action import ForwardCommand

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


class SendTargetsToJTC(Node):

    def __init__(self):
        super().__init__('action_client_node')

        # bringup_config_path = os.path.join(get_package_share_directory('pm_robot_bringup'), 'config/pm_robot_bringup_config.yaml')
        
        # f = open(bringup_config_path)
        # bringup_config = yaml.load(f,Loader=SafeLoader)
        # f.close()

        # self.callback_group = ReentrantCallbackGroup()
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self._action_client = ActionClient(self, ForwardCommand, 'trajectory_target', callback_group=self.group1)

        # Create the goals
        self.goal_1 = ForwardCommand.Goal()
        self.goal_1.targets = [-0.3, 0.0, 0.0]  # Example targets

        self.goal_2 = ForwardCommand.Goal()
        self.goal_2.targets = [0.0, 0.0, 0.0]  # Example targets

        self.goals = []
        self.goals.append(self.goal_1)
        self.goals.append(self.goal_2)

        self.i = 0

        self.goal_reached = True

        wait_sec_between_publish = 1
       
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback, callback_group=self.group2)

            
    def timer_callback(self):
 
        if self.goal_reached == True:
            if self.i >= len(self.goals):
                self.i = 0

            self.goal_reached = False

            self.send_goal_and_wait(self.goals[self.i]) 

            self.i = self.i + 1
        else:
            return
        

    def send_goal_and_wait(self, goal):
        self.get_logger().info(f"Sending goal: {goal}")

        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(goal)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):

        self.get_logger().info(f'Goal reached!')
        self.goal_reached = True



def main(args=None):
    rclpy.init(args=args)


    try:
        action_client_node = SendTargetsToJTC()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(action_client_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_client_node.destroy_node()

    finally:
        rclpy.shutdown()

   

if __name__ == '__main__':
    main()

