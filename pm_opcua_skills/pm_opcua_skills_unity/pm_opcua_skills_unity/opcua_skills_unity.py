import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils

from pm_opcua_skills_msgs.srv import ForceSensingMove
from std_msgs.msg import Float64MultiArray


class PmOpcuaSkillsUnityNode(Node):
    def __init__(self):
        super().__init__('pm_opcua_skills_unity_node')

        self.get_logger().info('PmOpcuaSkillsUnityNode has been started.')

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.pm_robot_utils = PmRobotUtils(self)
     
        # Clients 
        self.client_force_sensor = self.create_subscription(Float64MultiArray, '/pm_sensor_controller/ForceSensor/Stream',self.force_sensor_callback, 10)

        # Services
        self.force_sensing_move_srv = self.create_service(ForceSensingMove, 'pm_opcua_skills_unity/force_sensing_move', self.force_sensing_move_callback, callback_group=self.callback_group)

        self._current_force_sensor_data = Float64MultiArray()


    def force_sensing_move_callback(self, request:ForceSensingMove.Request, response:ForceSensingMove.Response):

        self.get_logger().info('Received ForceSensingMove request.')

        # Validate the request parameters. If any max force is > than 10, set threshold_exceeded to True and return failure.
        threshold_value = 10.0  # N
        if abs(request.max_fx) > threshold_value or abs(request.max_fy) > threshold_value or abs(request.max_fz) > threshold_value:
            self.get_logger().error('Max force exceeded the threshold of 10N.')
            response.success = False
            response.threshold_exceeded = True
            response.error = 'Max force exceeded the threshold of 10N.'
            return response
        

        # move to start position
        success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
            request.start_x,
            request.start_y,
            request.start_z,
            time= 1.0
        )

        # Check if the move to start position was successful
        if not success:
            self.get_logger().error('Failed to move to start position.')
            response.success = False
            response.error = 'Failed to move to start position.'
            return response
        self.get_logger().info('Moved to start position successfully.')

        # Check if the force sensor data is available and compare it with the threshold. If the force is below the threshold, move to the target position.
        if not self._current_force_sensor_data.data:
            self.get_logger().error('No force sensor data available.')
            response.success = False
            return response
        self.get_logger().info(f'Current force sensor data: {self._current_force_sensor_data.data}')

        step_size = request.step_size*1e-6  # Convert step size from micrometers to meters
        current_position = [request.start_x, request.start_y, request.start_z]
        target_position = [request.target_x, request.target_y, request.target_z]

        while current_position != target_position:
            # Check if the force sensor data exceeds the thresholds
            for i, (force, max_force, axis) in enumerate(zip(self._current_force_sensor_data.data, [request.max_fx, request.max_fy, request.max_fz], ['X', 'Y', 'Z'])):
                if abs(force) > max_force:
                    self.get_logger().warn(f'Force in {axis} direction exceeded threshold: {force} > {max_force}')
                    response.success = True
                    response.threshold_exceeded = True
                    return response

            # Calculate the next step position
            next_position = [
                min(current_position[0] + step_size, target_position[0]),
                min(current_position[1] + step_size, target_position[1]),
                min(current_position[2] + step_size, target_position[2]),
            ]

            self.get_logger().info(f'Moving to position: {next_position}')

            # Move to the next step position
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                next_position[0],
                next_position[1],
                next_position[2],
                time=1.0
            )

            if not success:
                self.get_logger().error('Failed to move to the next position.')
                response.success = False
                response.error = 'Failed to move to the next position.'
                return response

            # Update the current position
            current_position = next_position

        self.get_logger().info('Target position reached. Nothing found.')
        response.success = False
        response.error = 'Target position reached. Nothing found.'
        response.threshold_exceeded = False
        return response

    def force_sensor_callback(self, msg: Float64MultiArray):
        self._current_force_sensor_data = msg



def main(args=None):
    rclpy.init(args=args)

    node = PmOpcuaSkillsUnityNode()

    executor = MultiThreadedExecutor(num_threads=6) 

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
