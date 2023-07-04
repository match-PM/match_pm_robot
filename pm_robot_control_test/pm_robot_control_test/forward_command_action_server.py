import rclpy
import time
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import yaml

from pm_robot_interfaces.action import ForwardCommand

class ForwardCommandActionServer(Node):

    def __init__(self):
        super().__init__('forward_command_action_server')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        
        self.declare_parameter("controller_param", "")
        self.declare_parameter("robot_description", '')

        controller_name = 'pm_robot_xyz_axis_controller'

        self.robot_description = self.get_parameter("robot_description").value
        robot_control_yaml = self.get_parameter("controller_param").get_parameter_value().string_value

        with open(robot_control_yaml, 'r') as file:
            yaml_data = yaml.safe_load(file)

        self.joint_names = yaml_data[controller_name]['ros__parameters']['joints']

        self.joint_state_msg = JointState()
        self.goal = None

        publish_topic = "/" + controller_name + "/" + "commands"
        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        
        subscribe_topic = "/joint_states"
        self.subsriber = self.create_subscription(JointState, subscribe_topic, self.joint_state_callback, 10, callback_group=self.group1)

        self._action_server = ActionServer(
            self,
            ForwardCommand,
            'forward_command',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=self.group2)
        
        self.get_logger().info("ForwardCommand action server has been initialised")

        

    # Callback function for JointStates, saves states to joint_state_msg
    def joint_state_callback(self, msg):
        self.joint_state_msg = msg


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin action
        self.get_logger().info('Recieved goal request...')
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_request):
        # Accepts or rejects a client request to cancel action
        self.get_logger().info('Recieved cancel request...')
        self.goal = goal_request
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        
        result = ForwardCommand.Result()

        for idx, joint in enumerate(self.joint_names):
            # Checks, if the targets are in the joint limits
            lower, upper = get_joint_limits(ET.fromstring(self.robot_description), joint)
            #self.get_logger().warn(f'Joint {joint} with lower limit {lower} and upper limit {upper}.')
            if (lower is None and upper is None) or self.goal.targets[idx] > float(upper) or self.goal.targets[idx] < float(lower):
                self.get_logger().warn(f'Joint {joint} not found or target out of joint limits!')
                goal_handle.abort()
                result.goal_reached = False 
                return result
    
        
        targets = self.goal.targets

        msg = Float64MultiArray()
        msg.data = targets
        self.publisher_.publish(msg)

        goal_reached = []
        for joint in self.joint_names:
            goal_reached.append(False)
        

        while not all(goal_reached):
            for idx, joint in enumerate(self.joint_names):
                i = self.joint_state_msg.name.index(joint)
                lower_threshold = (self.goal.targets[idx]-10**-6)#*0.95 
                upper_threshold = (self.goal.targets[idx]+10**-6)#*1.05
                self.get_logger().info(f'lower threshold = {lower_threshold}, upper threshold = {upper_threshold}')
                if self.joint_state_msg.position[i] >= lower_threshold and self.joint_state_msg.position[i] < upper_threshold:
                    goal_reached[idx] = True
                    self.get_logger().info(f'Goal {idx} with {joint} reached')
                else:
                    self.get_logger().warn(f'Goal {idx} with{joint} not reached. Position: {self.joint_state_msg.position[i]}')
                time.sleep(0.1)

        goal_handle.succeed()
        # return the result

        result.goal_reached = True 
        return result
    

def get_joint_limits(robot_description: ET.Element, joint_name: str):
    # Iterate over all 'joint' elements in the URDF
    for joint in robot_description.findall('.//joint'):
        if 'name' in joint.attrib and joint.attrib['name'] == joint_name:
            limit = joint.find('limit')
            if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                return limit.attrib['lower'], limit.attrib['upper']

    # If the joint was not found or has no limit, return None
    return None, None

def main(args=None):
    rclpy.init(args=args)

    try:
        forward_command_action_server = ForwardCommandActionServer()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(forward_command_action_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            forward_command_action_server.destroy_node()

    finally:
        rclpy.shutdown()



if __name__ == '__main__':
    main()

