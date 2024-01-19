import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from .MEDAQLib import MEDAQLib


class ConfocalPublisher(Node):
    publisher: rclpy.publisher.Publisher
    timer: rclpy.timer.Timer

    sensor_instance: MEDAQLib

    def __init__(self):
        super().__init__("confocal_publisher")
        self.publisher_ = self.create_publisher(Float64MultiArray, "data", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sensor_instance = MEDAQLib.CreateSensorInstByName("IFC2421")
        self.sensor_instance.OpenSensorTCPIP("169.254.168.150")

    def timer_callback(self):
        available = self.sensor_instance.DataAvail()
        _, scaled_data, _ = self.sensor_instance.TransferData(available)

        msg = Float64MultiArray()
        msg.data = scaled_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Data = {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    node = ConfocalPublisher()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
