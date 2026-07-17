class PmRobotMeasurementError(Exception):
    def __init__(self, message="Error occured measuring with the robot."):
        self.message = message
        super().__init__(self.message)
