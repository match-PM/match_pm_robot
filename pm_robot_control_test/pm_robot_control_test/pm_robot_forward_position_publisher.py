import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

# publisher for the forward_position_controller. Positions are defined in pm_robot_position_publisher.yaml


class PublisherForwardPosition(Node):
    def __init__(self):
        super().__init__("pm_robot_forward_position_publisher")
        # Declare all parameters
        self.declare_parameter("controller_name", "forward_position_controller")
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("goal_names", ["pos1", "pos2"])

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "commands"

        self.get_logger().info(
            f'Publishing {len(goal_names)} goals on topic "{publish_topic}"\
              every {wait_sec_between_publish} s'
        )

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.goals[self.i]
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.i += 1
        self.i %= len(self.goals)


def main(args=None):
    rclpy.init(args=args)

    publisher_forward_position = PublisherForwardPosition()

    rclpy.spin(publisher_forward_position)
    publisher_forward_position.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
