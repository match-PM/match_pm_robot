class PmRobotError(Exception):
    def __init__(self, message="Error occured controlling the robot."):
        self.message = message
        super().__init__(self.message)