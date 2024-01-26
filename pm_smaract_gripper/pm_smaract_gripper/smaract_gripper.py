import rclpy
from rclpy.node import Node

import smaract.ctl as ctl


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


class ConfocalPublisher(Node):
    def __init__(self):
        super().__init__("confocal_sensor")
        buffer = ctl.FindDevices()
        if len(buffer) == 0:
            self.get_logger().error("No MCS2 devices found.")
            exit(1)
        locator = buffer.split("\n")[0]

        self.handle = ctl.Open(locator)
        self.get_logger().info(f"Opened MCS2 device: {locator}")

        self.channel = 0
        self.move_mode = ctl.MoveMode.CL_ABSOLUTE

        # We start by setting some general channel properties.
        type = ctl.GetProperty_i32(self.handle, self.channel, ctl.Property.CHANNEL_TYPE)
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
            ctl.SetProperty_i32(self.handle, self.channel, ctl.Property.HOLD_TIME, 1000)
        elif type == ctl.ChannelModuleType.PIEZO_SCANNER_DRIVER:
            # Enable the amplifier.
            ctl.SetProperty_i32(
                self.handle, self.channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE
            )
            # The hold time specifies how long the position is actively held after reaching the target.
            # This property is also not persistent and set to zero by default.
            # A value of 0 deactivates the hold time feature, the constant ctl.HOLD_TIME_INFINITE sets the time to infinite.
            # (Until manually stopped by "Stop") Here we set the hold time to 1000 ms.
            ctl.SetProperty_i32(self.handle, self.channel, ctl.Property.HOLD_TIME, 1000)
        elif type == ctl.ChannelModuleType.MAGNETIC_DRIVER:
            # Enable the amplifier (and start the phasing sequence).
            ctl.SetProperty_i32(
                self.handle, self.channel, ctl.Property.AMPLIFIER_ENABLED, ctl.TRUE
            )


def main(args=None):
    rclpy.init(args=args)

    assert_lib_compatibility()

    node = ConfocalPublisher()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
