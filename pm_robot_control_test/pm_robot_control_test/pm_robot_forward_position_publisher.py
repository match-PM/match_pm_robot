import rclpy
from rclpy.node import Node
# Removed ParameterType, ParameterDescriptor imports as they are no longer needed for hardcoded position
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState

class PublisherForwardPosition(Node):
    def __init__(self):
        super().__init__("pm_robot_forward_position_publisher")

        # Declare parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 5)
        
        # Hardcode the target position directly in the node
        # Ensure the number of elements matches your robot's joints (e.g., 3 for X, Y, Z Axis Joints)
        self.target_position = [0.1, 0.2, 0.3] # Example hardcoded position

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        
        # Removed parameter reading and validation for target_position as it's now hardcoded

        publish_topic = "/" + controller_name + "/" + "commands"

        self.get_logger().info(
            f'Publishing hardcoded goal "{self.target_position}" on topic "{publish_topic}" '
            f'every {wait_sec_between_publish} seconds.'
        )

        self.publisher_ = self.create_publisher(JointTrajectoryControllerState, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    def timer_callback(self):
        """
        Callback function for the timer.
        Publishes the single hardcoded target position repeatedly.
        """
        msg = JointTrajectoryControllerState()
        msg.desired = self.target_position
        self.get_logger().info(f'Publishing: "{msg.desired}"')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        publisher_forward_position = PublisherForwardPosition()
        rclpy.spin(publisher_forward_position)
    except Exception as e:
        rclpy.logging.get_logger("pm_robot_forward_position_publisher").error(f"Error: {e}")
    finally:
        if 'publisher_forward_position' in locals() and publisher_forward_position:
            publisher_forward_position.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
