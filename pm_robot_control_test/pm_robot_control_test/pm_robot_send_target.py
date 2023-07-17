import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

class SendPosition(Node):

    def __init__(self):
        super().__init__('send_position')

        self._action_client = ActionClient(self,FollowJointTrajectory,'/pm_robot_xyz_axis_controller/follow_joint_trajectory')

        subscribe_topic = "/joint_states"
        self.subsriber = self.create_subscription(JointState, subscribe_topic, self.joint_state_callback, 10)
            

    def send_target(self,points):
        
        point = JointTrajectoryPoint()
        point.positions = points
        point.velocities = [0.01, 0.01 , 0.01]
        # joint_trajectory.accelerations = [1.0, 10.0 , 5.0, 1.0]

        goal_msg = FollowJointTrajectory.Goal()
        
        #point.time_from_start = Duration(sec=4)
        
        goal_msg.trajectory.joint_names = ['X_Axis_Joint','Y_Axis_Joint','Z_Axis_Joint']

        goal_msg.trajectory.points = [point]

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self.get_logger().info(f'Future: "{self._action_client._feedback_callbacks.values()}"')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f'Publishing goals "{point.positions}"')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        error_code = future.result().result.error_code
        if error_code != 0:
            self.get_logger().info(f'Error code: "{error_code}"')
        self.get_logger().info(f'Goal reached!')
        rclpy.shutdown()

    
    # Callback function for JointStates, saves states to joint_state_msg
    def joint_state_callback(self, msg):
        self.joint_state_msg = msg

    
    
def main(args=None):
    rclpy.init(args=args)

    action_client = SendPosition()
    action_client.send_target([-0.3, -0.0, 0.0])
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()

