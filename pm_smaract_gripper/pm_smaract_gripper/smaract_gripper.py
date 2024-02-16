import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import ctl

from pm_msgs.action import SmaractMove


def assert_lib_compatibility():
    """
    Checks that the major version numbers of the Python API and the
    loaded shared library are the same to avoid errors due to
    incompatibilities.
    Raises a RuntimeError if the major version numbers are different.
    """
    vapi = ctl.api_version
    vlib = [int(i) for i in ctl.GetFullVersionString().split(".")]
    if vapi[0] != vlib[0]:
        raise RuntimeError("Incompatible SmarActCTL python api and library version.")


class SmaractGripper(Node):
    def __init__(self):
        super().__init__("pm_smaract_gripper")
        buffer = ctl.FindDevices()
        if len(buffer) == 0:
            self.get_logger().error("No MCS2 devices found.")
            exit(1)
        locator = buffer.split("\n")[0]

        self.handle = None

        try:
            self.handle = ctl.Open(locator)
            self.get_logger().info(f"Opened MCS2 device: {locator}")

            self.channel = 0
            self.move_mode = ctl.MoveMode.CL_ABSOLUTE

            # We start by setting some general channel properties.
            type = ctl.GetProperty_i32(
                self.handle, self.channel, ctl.Property.CHANNEL_TYPE
            )
            if type == ctl.ChannelModuleType.STICK_SLIP_PIEZO_DRIVER:
                # Set max closed loop frequency (maxCLF) to 6 kHz. This properties sets a limit for the maximum actuator driving frequency.
                # The maxCLF is not persistent thus set to a default value at startup.
                ctl.SetProperty_i32(
                    self.handle, self.channel, ctl.Property.MAX_CL_FREQUENCY, 6000
                )
                # The hold time specifies how long the position is actively held after reaching the target.
                # This property is also not persistent and set to zero by default.
                # A value of 0 deactivates the hold time feature, the constant ctl.HOLD_TIME_INFINITE sets the time to infinite.
                # (Until manually stopped by "Stop") Here we set the hold time to 1000 ms.
                ctl.SetProperty_i32(
                    self.handle, self.channel, ctl.Property.HOLD_TIME, 1000
                )
            elif type == ctl.ChannelModuleType.PIEZO_SCANNER_DRIVER:
                # Enable the amplifier.
                ctl.SetProperty_i32(
                    self.handle, self.channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE
                )
                # The hold time specifies how long the position is actively held after reaching the target.
                # This property is also not persistent and set to zero by default.
                # A value of 0 deactivates the hold time feature, the constant ctl.HOLD_TIME_INFINITE sets the time to infinite.
                # (Until manually stopped by "Stop") Here we set the hold time to 1000 ms.
                ctl.SetProperty_i32(
                    self.handle, self.channel, ctl.Property.HOLD_TIME, 1000
                )
            elif type == ctl.ChannelModuleType.MAGNETIC_DRIVER:
                # Enable the amplifier (and start the phasing sequence).
                ctl.SetProperty_i32(
                    self.handle, self.channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE
                )
        except Exception as e:
            self.get_logger().error(f"{e}")
            exit(1)

        self.move_mode = ctl.MoveMode.CL_ABSOLUTE
        ctl.SetProperty_i32(
            self.handle, self.channel, ctl.Property.MOVE_MODE, self.move_mode
        )

        self.move_action = ActionServer(self, SmaractMove, "~/move", self.move_callback)

    def move_callback(self, goal_handle):
        goal = goal_handle.request.goal
        self.get_logger(f"Received new move goal: {goal}")

        # Set move velocity [in pm/s].
        ctl.SetProperty_i64(
            self.handle, self.channel, ctl.Property.MOVE_VELOCITY, 1000000000
        )
        # Set move acceleration [in pm/s2].
        ctl.SetProperty_i64(
            self.handle, self.channel, ctl.Property.MOVE_ACCELERATION, 1000000000
        )
        ctl.Move(self.handle, self.channel, goal, 0)

        goal_handle.succeed()
        result = SmaractMove.Result()
        result.final_position = 0.0
        return result


def main(args=None):
    rclpy.init(args=args)

    assert_lib_compatibility()

    node = SmaractGripper()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
