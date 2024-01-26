import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from .MEDAQLib import MEDAQLib

SENSOR_NAME = "IFC2421"
SENSOR_ADDRESS = "169.254.168.150"


class ConfocalPublisher(Node):
    publisher: rclpy.publisher.Publisher
    timer: rclpy.timer.Timer

    sensor_instance: MEDAQLib

    def __init__(self):
        super().__init__("confocal_sensor")

        self.get_logger().info(
            f"Establishing connection to {SENSOR_NAME} at address {SENSOR_ADDRESS}"
        )
        self.sensor_instance = MEDAQLib.CreateSensorInstByName(SENSOR_NAME)
        self.sensor_instance.OpenSensorTCPIP(SENSOR_ADDRESS)

        err = self.get_error()
        if len(err) > 0:
            self.get_logger().error(f"Failed to connect to sensor: {err}")
        else:
            self.publisher_ = self.create_publisher(Float64MultiArray, "~/data", 1000)
            self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        available = self.sensor_instance.DataAvail()
        if available == 0:
            return

        _, scaled_data, _ = self.sensor_instance.TransferData(available)

        err = self.get_error()
        if len(err) > 0:
            self.get_logger().error(f"Failed to transfer data: {err}")

        msg = Float64MultiArray()
        msg.data = scaled_data
        self.publisher_.publish(msg)

    def get_error(self):
        err = self.sensor_instance.GetError()
        end = err.find("\0")
        return err[:end]


def main(args=None):
    rclpy.init(args=args)

    node = ConfocalPublisher()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
