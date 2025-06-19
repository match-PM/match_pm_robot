import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from math import sqrt

from pm_opcua_skills_msgs.srv import ForceSensingMove, Dispense
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
        self.force_sensing_move_srv = self.create_service(ForceSensingMove, 'pm_opcua_skills_controller/ForceSensingMove', self.force_sensing_move_callback, callback_group=self.callback_group)
        # self.dispense_move_srv = self.create_service(Dispense, 'pm_opcua_skills_controller/Dispense', self.dispense_move_callback, callback_group=self.callback_group)

        self._current_force_sensor_data = Float64MultiArray()


    # def dispense_move_callback(self, request:Dispense.Request, response:Dispense.Response):
    #     self.get_logger().info('Received Dispense request.')

    #     time = request.time
    #     z_height = request.height
    #     z_move = request.move

    #     current_x = self.pm_robot_utils.get_current_joint_state('X_Axis_Joint')
    #     current_y = self.pm_robot_utils.get_current_joint_state('Y_Axis_Joint')
    #     current_z = self.pm_robot_utils.get_current_joint_state('Z_Axis_Joint')

    #     self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
    #         current_x,
    #         current_y,
    #         z_height,
    #         time=time
    #     )



    def force_sensing_move_callback(self, request:ForceSensingMove.Request, response:ForceSensingMove.Response):

        self.get_logger().info('Received ForceSensingMove request.')

        # Validate the request parameters. If any max force is > than 10, set threshold_exceeded to True and return failure.
        threshold_value = 10.0  # N
        max_step_size = 100  # micrometers
        max_steps = 1000  # maximum number of steps


        if abs(request.max_fx) > threshold_value or abs(request.max_fy) > threshold_value or abs(request.max_fz) > threshold_value:
            self.get_logger().error('Max force exceeded the threshold of 10N.')
            response.success = False
            response.threshold_exceeded = True
            response.error = 'Max force exceeded the threshold of 10N.'
            return response
        
        if request.step_size > max_step_size:
            self.get_logger().error(f'Step size {request.step_size} micrometers exceeds the maximum allowed step size of {max_step_size} micrometers.')
            response.success = False
            response.error = f'Step size exceeds the maximum allowed step size of {max_step_size} micrometers.'
            return response

        step_size = request.step_size*1e-6  # Convert step size from micrometers to meters
        current_position = [request.start_x, request.start_y, request.start_z]
        target_position = [request.target_x, request.target_y, request.target_z]

        length_x = request.target_x - request.start_x
        length_y = request.target_y - request.start_y
        length_z = request.target_z - request.start_z
        # Calculate the length of the vector from start to target position
        length = sqrt(length_x**2 + length_y**2 + length_z**2)

        if (length/ step_size) > max_steps:
            self.get_logger().error(f'The distance to the target position is too large. The maximum number of steps is {max_steps}.')
            response.success = False
            response.error = f'The distance to the target position is too large. The maximum number of steps is {max_steps}.'
            return response

        # move to start position
        success_xyz = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
            request.start_x,
            request.start_y,
            request.start_z,
            time= 1.0
        )

        success_t = self.pm_robot_utils.send_t_trajectory_goal_absolut(
            request.start_t,
            time=1.0
        )

        # Check if the move to start position was successful
        if not success_xyz or not success_t:
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

        step_size_x = step_size * length_x / length
        step_size_y = step_size * length_y / length
        step_size_z = step_size * length_z / length

        while current_position != target_position:
            # Check if the force sensor data exceeds the thresholds
            for i, (force, max_force, axis) in enumerate(zip(self._current_force_sensor_data.data, [request.max_fx, request.max_fy, request.max_fz], ['X', 'Y', 'Z'])):
                if abs(force) > max_force:
                    self.get_logger().warn(f'Force in {axis} direction exceeded threshold: {force} > {max_force}')
                    response.success = True
                    response.threshold_exceeded = True
                    return response

            step_target = [
                current_position[0] + step_size_x,
                current_position[1] + step_size_y,
                current_position[2] + step_size_z,
            ]

            self.get_logger().info(f'Moving to position: {step_target}')

            # Move to the next step position
            success = self.pm_robot_utils.send_xyz_trajectory_goal_absolut(
                step_target[0],
                step_target[1],
                step_target[2],
                time=1.0
            )

            if not success:
                self.get_logger().error('Failed to move to the next position.')
                response.success = False
                response.error = 'Failed to move to the next position.'
                return response

            # Update the current position
            current_position = step_target

            # calculate the distance to the target position
            distance_to_target = sqrt(
                (target_position[0] - current_position[0])**2 +
                (target_position[1] - current_position[1])**2 +
                (target_position[2] - current_position[2])**2
            )
            if distance_to_target < step_size:
                self.get_logger().info('Reached the target position.')
                break

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
