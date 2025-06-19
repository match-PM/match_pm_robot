import rclpy
from rclpy.node import Node
import time

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import rclpy.time

from pm_moveit_interfaces.srv import MoveToPose,  MoveToFrame, MoveRelative
from pm_robot_primitive_skills.py_modules.adhesive_test_points import TestPointGrid
import pm_moveit_interfaces.srv as pm_moveit_srv
import pm_msgs.srv as pm_msg_srv
from example_interfaces.srv import SetBool
from std_msgs.msg import Float64MultiArray
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from math import sqrt

# import empty_with_success from pm_msgs
from pm_msgs.srv import EmptyWithSuccess

#import copy
import copy

class PrimitiveSkillsNode(Node):
    DISPENSER_TRAVEL_DISTANCE = 0.04
    DISPENSER_OFFSET_VALUE = 0.01
    DEFAULT_DISPENSE_HEIGHT = 0.00001
    
    def __init__(self):
        super().__init__('pm_robot_primitive_skills')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        sim_time = self.get_parameter('use_sim_time').value
        
        self.logger = self.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()
        
        self.adhesive_test_point_grid_1 = TestPointGrid(ind_of_test_grid=0)
        self.adhesive_test_point_grid_2 = TestPointGrid(ind_of_test_grid=1)

        self.pm_robot_utils = PmRobotUtils(self)

        self._current_force_sensor_data = Float64MultiArray()
        
        # create services
        self.srv = self.create_service(pm_msg_srv.DispenseForTime, self.get_name()+'/dispense_1K', self.dispense_callback)

        self.dispense_test_point_srv = self.create_service(pm_msg_srv.EmptyWithSuccess, self.get_name()+'/dispense_test_point', self.dispense_test_point_callback)
        self.reset_test_station_srv = self.create_service(pm_msg_srv.EmptyWithSuccess, self.get_name()+'/reset_test_station', self.reset_test_station_callback)
        self.dispense_at_frames_srv = self.create_service(pm_msg_srv.DisppenseAtPoints, self.get_name()+'/dispense_at_frames', self.dispense_at_points_callback)
        self.force_sensing_move_srv = self.create_service(pm_msg_srv.ForceSensingMove, self.get_name()+'/ForceSensingMove', self.force_sensing_move_callback, callback_group=self.callback_group_mu_ex)
        self.logger.info("Primitive skills node started!")
        
        # create clients
        self.client_force_sensor = self.create_subscription(Float64MultiArray, '/pm_sensor_controller/ForceSensor/Stream',self.force_sensor_callback, 10)
        self.open_protection_gaz_srv = self.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint',callback_group = self.callback_group_re)
        self.dispenser_joint_gaz_srv = self.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Joint',callback_group = self.callback_group_re)
        
        self.open_protection_real_srv = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveBackward',callback_group = self.callback_group_re)
        self.close_protection_real_srv = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveForward',callback_group = self.callback_group_re)

        self.retract_dispenser_real_srv = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveBackward',callback_group = self.callback_group_re)
        self.extend_dispenser_real_srv = self.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveForward',callback_group = self.callback_group_re)
                
        self.create_adhesive_viz_point_srv = self.create_client(pm_msg_srv.CreateVizAdhesivePoint, '/pm_adhesive_displayer/add_point',callback_group = self.callback_group_re)

        self.client_dips_1k_on = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
        self.client_dips_1k_off = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)
        
        self.move_robot_tool_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_1k_dispenser_to_frame',callback_group=self.callback_group_re)

        if not sim_time:
            # self.dispense_1K = self.create_service(DispenseForTime, self.get_name()+'/dispense_1K', self.dispense_callback)
            self.move_uv_in_curing_position_service = self.create_service(SetBool, self.get_name()+"/move_uv_in_curing_position", self.move_uv_in_curing_position_service_callback,callback_group=self.callback_group_mu_ex)
            self.uv_curing = self.create_service(pm_msg_srv.UVCuringSkill, self.get_name()+'/uv_curing', self.uv_curing_callback, callback_group=self.callback_group_mu_ex)

            self.client_uv_front_forward= self.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Front_Joint/MoveForward",callback_group = self.callback_group_re)
            self.client_uv_front_backward= self.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Front_Joint/MoveBackward",callback_group = self.callback_group_re)
            self.client_uv_back_forward= self.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Back_Joint/MoveForward",callback_group = self.callback_group_re)
            self.client_uv_back_backward= self.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Back_Joint/MoveBackward",callback_group = self.callback_group_re)

            self.client_dips_1k_on = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
            self.client_dips_1k_off = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)

            self.client_uv_controller_on = self.create_client(pm_msg_srv.UVSetOnOff, '/pm_uv_controller/Hoenle_UV/SetOnOff',callback_group = self.callback_group_re)
            self.client_uv_controller_time = self.create_client(pm_msg_srv.UVSetTime, '/pm_uv_controller/Hoenle_UV/SetTime',callback_group = self.callback_group_re)
            self.client_uv_controller_intensity = self.create_client(pm_msg_srv.UVSetPower, '/pm_uv_controller/Hoenle_UV/SetPower',callback_group = self.callback_group_re)

        self.logger.info(f"Primitive skills node started! Using sim time: {sim_time}")
        
    
    def force_sensing_move_callback(self, request:pm_msg_srv.ForceSensingMove.Request, response:pm_msg_srv.ForceSensingMove.Response):

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


    def uv_curing_callback(self, request: pm_msg_srv.UVCuringSkill.Request, response:pm_msg_srv.UVCuringSkill.Response):
        self.logger.info("UV curing callback called!")
        try:
            intensity_request = pm_msg_srv.UVSetPower.Request()
            intensity_request.power = request.intensity_percent

            if not self.client_uv_controller_intensity.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_uv_controller/Hoenle_UV/SetPower' not available")
                response.success = False
                return response
            
            self.client_uv_controller_intensity.call(intensity_request)
            self.logger.info(f"Set UV intensity to {request.intensity_percent}%")

            time_request = pm_msg_srv.UVSetTime.Request()
            time_request.time = request.duration

            if not self.client_uv_controller_time.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_uv_controller/Hoenle_UV/SetTime' not available")
                response.success = False
                return response
            
            self.client_uv_controller_time.call(time_request)
            self.logger.info(f"Set UV time to {request.duration} seconds")

            on_request = pm_msg_srv.UVSetOnOff.Request()
            on_request.turn_on = [True, True, True, True]

            if not self.client_uv_controller_on.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_uv_controller/Hoenle_UV/SetOnOff' not available")
                response.success = False
                return response
            
            self.client_uv_controller_on.call(on_request)
            self.logger.info("Turned on UV LEDs")

            # sleep for the duration of the UV curing, take the highest value of the duration array
            time.sleep(max(request.duration))

            response.success = True
            response.message = "UV curing successful!"

        except Exception as e:
            self.logger.error(f"Error in uv_curing_callback: {e}")
            response.success = False
            response.message = "Error in uv_curing_callback"
            return response

        return response


    def move_dispenser_to_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request)-> bool:
        call_async = False

        if not self.move_robot_tool_client.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_moveit_server/move_tool_to_frame' not available")
            return False
        
        req = move_to_frame_request

        if call_async:
            future = self.move_robot_tool_client.call_async(req)
            
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is None:
                self.logger.error("Service call failed or timeout occurred.")
                return False

            return future.result().success
        
        else:
            response:pm_moveit_srv.MoveToFrame.Response = self.move_robot_tool_client.call(req)
            return response.success 
    
    def create_adhesive_viz_point(self, request: pm_msg_srv.CreateVizAdhesivePoint.Request)->bool:
        if not self.create_adhesive_viz_point_srv.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/adhesive_display_node/add_point' not available")
            return False
        
        response:pm_msg_srv.CreateVizAdhesivePoint.Response = self.create_adhesive_viz_point_srv.call(request)
        return response.success
    
    def move_uv_in_curing_position_service_callback(self, request:SetBool.Request, response:SetBool.Response):
        """Moves both UV LEDs in curing position"""
        
        response.success = self.move_uv_in_curing_position(request)
        response.message = "UV LEDs moved in curing position!" if response.success else "Failed to move UV LEDs in curing position!"

        return response
    
    def move_uv_in_curing_position(self, request:SetBool.Request)->bool:
        
        if not self.client_uv_front_forward.wait_for_service(timeout_sec=2.0) and not self.client_uv_front_backward.wait_for_service(timeout_sec=2.0) and not self.client_uv_back_forward.wait_for_service(timeout_sec=2.0) and not self.client_uv_back_backward.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Services not available!")
            return False

        request_empty = pm_msg_srv.EmptyWithSuccess.Request()

        if request.data == True:
            success_front = self.client_uv_front_forward.call(request_empty)
            success_back = self.client_uv_back_forward.call(request_empty)
        else:
            success_front = self.client_uv_front_backward.call(request_empty)
            success_back = self.client_uv_back_backward.call(request_empty)

        self.get_logger().info(f"Service call result: success={success_front}")
        self.get_logger().info(f"Service call result: success={success_back}")

        if success_front and success_back:
            return True
        else:
            return False

    def open_protection(self):
        
        if self.is_gazebo_running():
            if not self.open_protection_gaz_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint' not available")
                return False
            
            req = SetBool.Request()
            req.data = True
            response:SetBool.Response = self.open_protection_gaz_srv.call(req)
            
            return response.success
        
        else:
            if not self.open_protection_real_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveBackward' not available")
                return False
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.open_protection_real_srv.call(req)
            
            return response.success
    
    def close_protection(self):
        
        if self.is_gazebo_running():
            if not self.open_protection_gaz_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint' not available")
                return False
            
            req = SetBool.Request()
            req.data = False
            response:SetBool.Response = self.open_protection_gaz_srv.call(req)
            return response.success
        else:
            if not self.close_protection_real_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveForward' not available")
                return False
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.close_protection_real_srv.call(req)
            return response.success
    
    def retract_dispenser(self):
        
        if self.is_gazebo_running():
            if not self.dispenser_joint_gaz_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Joint' not available")
                return False
            
            req = SetBool.Request()
            req.data = True
            response:SetBool.Response = self.dispenser_joint_gaz_srv.call(req)
            return response.success
        else:
            if not self.retract_dispenser_real_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_controller/1K_Dispenser_Joint/MoveBackward' not available")
                return False
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.retract_dispenser_real_srv.call(req)
            return response.success
    
    def extend_dispenser(self):
        if self.is_gazebo_running():
            if not self.dispenser_joint_gaz_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Joint' not available")
                return False
            
            req = SetBool.Request()
            req.data = False
            response:SetBool.Response = self.dispenser_joint_gaz_srv.call(req)
            return response.success
        else:
            if not self.extend_dispenser_real_srv.wait_for_service(timeout_sec=1.0):
                self.logger.error("Service '/pm_pneumatic_controller/1K_Dispenser_Joint/MoveForward' not available")
                return False
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.extend_dispenser_real_srv.call(req)
            return response.success
    
    def dispense_callback(self, request: pm_msg_srv.DispenseForTime.Request, response:pm_msg_srv.DispenseForTime.Response):
        
        self.logger.warn(f"{request.time}")

        if not self.client_dips_1k_on.wait_for_service(timeout_sec=1.0):
            self.logger.error("Client '/pm_nozzle_controller/Doseur_Nozzle/Pressure' not available!")
            response.success = False
            return response

        if not self.client_dips_1k_off.wait_for_service(timeout_sec=1.0):
            self.logger.error("Client '/pm_nozzle_controller/Doseur_Nozzle/TurnOff' not available!")
            response.success = False
            return response
        
        request_on = pm_msg_srv.EmptyWithSuccess.Request()

        response_on:pm_msg_srv.EmptyWithSuccess.Response = self.client_dips_1k_on.call(request_on)
        
        # future = self.client_dips_1k_on.call_async(request_on)

        # while not future.done():
        #     rclpy.spin_once(self)

        #response_on:pm_msg_srv.EmptyWithSuccess.Response = future.result()

        if not response_on.success:
            response.success = False
            return response
        
        # This does not work yet --> find adtequate function for waiting
        self.create_rate(0.1)
        #time.sleep(request.time)

        request_off = pm_msg_srv.EmptyWithSuccess.Request()

        response_off:pm_msg_srv.EmptyWithSuccess.Response = self.client_dips_1k_off.call(request_off)

        # future_off = self.client_dips_1k_off.call_async(request_off)
        # while not future_off.done():
        #     rclpy.spin_once(self)
        # response_off:pm_msg_srv.EmptyWithSuccess.Response = future_off.result()
        
        if not response_off.success:
            response.success = False
            return response    
        
        response.success = True

        return response

    def dispense_at_points_callback(self, request: pm_msg_srv.DisppenseAtPoints.Request, response:pm_msg_srv.DisppenseAtPoints.Response):
        
        dispenser_prepared = False
        
        for frame in request.frame_names:
            move_to_frame_request = pm_moveit_srv.MoveToFrame.Request()
            
            move_to_frame_request.target_frame = frame
            move_to_frame_request.execute_movement = True
            
            if not dispenser_prepared:
                prepare_success = self.prepare_dispenser(move_to_frame_request)
                dispenser_prepared = True
                if not prepare_success:
                    self.logger.error("Preparing dispenser failed!")
                    response.success = False
                    return response
                
            success = self.dispense_at_frame(move_to_frame_request, frame)
            
            if not success:
                response.success = False
                return response
        
        retract_success = self.retract_dispenser()
        close_success = self.close_protection()
        
        if not retract_success or not close_success:
            response.success = False
            self.logger.error("Retracting dispenser or closing protection failed!")
            return response
        
        response.success = True
        return response
    
    def dispense_test_point_callback(self, request: pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        num = 5
        
        dispenser_prepared = False
        
        for i in range(num):

            if not self.adhesive_test_point_grid_1.is_station_saturated():
                current_station = self.adhesive_test_point_grid_1
                self.logger.info(f"Dispensing test point {current_station.get_current_point()}/{current_station._max_points} at station 1!")
            elif not self.adhesive_test_point_grid_2.is_station_saturated():
                self.logger.warn("Station 1 is saturated! Using station 2")
                current_station = self.adhesive_test_point_grid_2
                self.logger.info(f"Dispensing test point {current_station.get_current_point()}/{current_station._max_points} at station 2!")
            else:
                self.logger.warn("Both stations are saturated!")
                response.success = False
                return response
            
            move_to_frame_request = pm_moveit_srv.MoveToFrame.Request()
            move_to_frame_request.target_frame = current_station.frame_id
            move_to_frame_request.translation.x = float(current_station.get_start_offset().x)
            move_to_frame_request.translation.y = float(current_station.get_start_offset().y)
            move_to_frame_request.translation.z = float(current_station.get_start_offset().z)
            
            move_to_frame_request.translation.x += float(current_station.get_current_offset().x)
            move_to_frame_request.translation.y += float(current_station.get_current_offset().y)
            move_to_frame_request.translation.z += float(current_station.get_current_offset().z)
            
            move_to_frame_request.execute_movement = True
            
            if not dispenser_prepared:
                prepare_success = self.prepare_dispenser(move_to_frame_request)
                dispenser_prepared = True
                if not prepare_success:
                    self.logger.error("Preparing dispenser failed!")
                    response.success = False
                    return response
                
            success = self.dispense_at_frame(move_to_frame_request, "Test_Point_" + str(current_station.get_current_point()+1))
            response.success = success
            
            if success:
                current_station.increment_current_point()

            else:
                self.logger.error("Dispensing at frame failed!")
                return response
        time.sleep(1)
        
        success_retract_dispenser = self.retract_dispenser()
        success_close_protection = self.close_protection()
        time.sleep(1)
        
        return response
    
    def prepare_dispenser(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request)->bool:
        
        move_to_frame_request_copy = copy.deepcopy(move_to_frame_request)
        
        move_to_frame_request_copy.translation.z += self.DISPENSER_TRAVEL_DISTANCE
        
        move_to_frame_request_copy.translation.z += self.DISPENSER_OFFSET_VALUE
        
        move_success = self.move_dispenser_to_frame(move_to_frame_request_copy)
        
        if not move_success:
            return False 
        
        success_open_protection = self.open_protection()
        
        if not success_open_protection:
            return False
        
        success_extend_dispenser = self.extend_dispenser()
        
        time.sleep(0.5)
        
        return success_extend_dispenser
    
    def dispense_at_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request, point_name: str)->bool:
        
        move_to_frame_request.translation.z += float(self.DEFAULT_DISPENSE_HEIGHT)
        move_to_frame_request.translation.z += self.DISPENSER_OFFSET_VALUE

        success = self.move_dispenser_to_frame(move_to_frame_request)
        
        if not success:
            return False
        
        move_to_frame_request.translation.z -= self.DISPENSER_OFFSET_VALUE
        
        success = self.move_dispenser_to_frame(move_to_frame_request)
        
        if not success:
            return False
        
        create_adhesive_viz_point_request = pm_msg_srv.CreateVizAdhesivePoint.Request()
        create_adhesive_viz_point_request.point.parent_frame = move_to_frame_request.target_frame
        
        create_adhesive_viz_point_request.point.hight = 1.0 # in mm
        create_adhesive_viz_point_request.point.diameter = 1.0 # in mm
        create_adhesive_viz_point_request.point.name = point_name
        success_create_viz_point = self.create_adhesive_viz_point(create_adhesive_viz_point_request)

        move_to_frame_request.translation.z += self.DISPENSER_OFFSET_VALUE
        
        success = self.move_dispenser_to_frame(move_to_frame_request)

        return success
    
    def reset_test_station_callback(self, request: pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        self.adhesive_test_point_grid_1.reset_current_point()
        self.adhesive_test_point_grid_2.reset_current_point()
        response.success = True
        return response
    
    def is_gazebo_running(self):
        """Check if the Gazebo node is active."""
        node_names = self.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False
    
    
    def is_unity_running(self)->bool:
        """Check if the Unity node is active."""
        node_names = self.get_node_names()
        if 'ROS2UnityCam1Publisher' in node_names:
            return True
        return False
    
def main(args=None):
    rclpy.init(args=args)

    node = PrimitiveSkillsNode()

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
