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
import pm_msgs.msg as pm_msg_msg
from example_interfaces.srv import SetBool
from std_msgs.msg import Float64MultiArray
from math import sqrt
from pm_uepsilon_confocal_msgs.srv import GetValue
from pm_opcua_skills_msgs.srv import Dispense
# import empty_with_success from pm_msgs
from pm_msgs.srv import EmptyWithSuccess
import copy
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_primitive_skills.py_modules.PrimitiveSkillsUtils import PrimitiveSkillsUtils

class PrimitiveSkillsNode(Node):
    
    def __init__(self):
        super().__init__('pm_robot_primitive_skills')

        self.pm_primitive_skills_utils = PrimitiveSkillsUtils(self)

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        sim_time = self.get_parameter('use_sim_time').value
        
        self.logger = self.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()
        
        self.adhesive_test_point_grid_1 = TestPointGrid(ind_of_test_grid=0)
        self.adhesive_test_point_grid_2 = TestPointGrid(ind_of_test_grid=1)
        
        # create services
        self.dispense_test_point_srv = self.create_service(pm_msg_srv.DispenseTestPoints, self.get_name()+'/dispense_test_point', self.dispense_test_point_callback)
        self.reset_test_station_srv = self.create_service(pm_msg_srv.EmptyWithSuccess, self.get_name()+'/reset_test_station', self.reset_test_station_callback)
        self.dispense_at_frames_srv = self.create_service(pm_msg_srv.DispenseAtPoints, self.get_name()+'/dispense_at_frames', self.dispense_at_points_callback)
        self.move_uv_in_curing_position_service = self.create_service(SetBool, self.get_name()+"/move_uv_in_curing_position", self.move_uv_in_curing_position_service_callback,callback_group=self.callback_group_mu_ex)
        self.uv_curing = self.create_service(pm_msg_srv.UVCuringSkill, self.get_name()+'/uv_curing', self.uv_curing_callback, callback_group=self.callback_group_mu_ex)
        
        self.get_confocal_top_measurement_srv = self.create_service(GetValue, self.get_name()+'/get_confocal_top_measurement', self.get_confocal_top_measurement_callback)
        self.get_confocal_bottom_measurement_srv = self.create_service(GetValue, self.get_name()+'/get_confocal_bottom_measurement', self.get_confocal_bottom_measurement_callback)
        
        self.set_uv_cart_position_srv = self.create_service(pm_msg_srv.SetUvSliderXPositions, self.get_name()+'/set_uv_slider_manual_positions', self.set_uv_cart_positions, callback_group=self.callback_group_mu_ex)
        
        self.uv_cart_publisher = self.create_publisher(Float64MultiArray,"/pm_uv_cart_manual_controller/commands",10)
        
        self.logger.info(f"Primitive skills node started! Using sim time: {sim_time}")
        

    def uv_curing_callback(self, request: pm_msg_srv.UVCuringSkill.Request, 
                           response:pm_msg_srv.UVCuringSkill.Response):
        
        self.logger.info("UV curing callback called!")
        try:
            intensity_request = pm_msg_srv.UVSetPower.Request()
            time_request = pm_msg_srv.UVSetTime.Request()
            on_request = pm_msg_srv.UVSetOnOff.Request()

            for index, spot_property in enumerate(request.uv_curing_spot_properties):
                spot_property: pm_msg_msg.UvCuringSpotProperty
                self.logger.info(f"Setting UV spot {index+1}: Intensity {spot_property.intensity_percent}%, Duration {spot_property.duration}s, Activation {spot_property.set_activation}")
                intensity_request.power[index] = spot_property.intensity_percent
                time_request.time[index] = spot_property.duration
                on_request.turn_on[index] = spot_property.set_activation

            # check if services are available, if not raise error
            if not self.pm_primitive_skills_utils.client_uv_controller_intensity.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service '/pm_uv_controller/Hoenle_UV/SetPower' not available")
            
            # call services to set intensity, time and on/off state of the UV LEDs
            self.pm_primitive_skills_utils.client_uv_controller_intensity.call(intensity_request)
            self.logger.info(f"Set UV intensity to {intensity_request.power} in %")

            # check if service is available, if not raise error
            if not self.pm_primitive_skills_utils.client_uv_controller_time.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service '/pm_uv_controller/Hoenle_UV/SetTime' not available")

            # call service to set time
            self.pm_primitive_skills_utils.client_uv_controller_time.call(time_request)
            self.logger.info(f"Set UV time to {time_request.time} seconds")

            # check if service is available, if not raise error
            if not self.pm_primitive_skills_utils.client_uv_controller_on.wait_for_service(timeout_sec=1.0):
                raise PmRobotError("Service '/pm_uv_controller/Hoenle_UV/SetOnOff' not available")

            # call service to turn on UV LEDs
            self.pm_primitive_skills_utils.client_uv_controller_on.call(on_request)
            
            self.logger.info("Turned on UV LEDs")

            # sleep for the duration of the UV curing, take the highest value of the duration array
            time.sleep(max(time_request.time)+1) # add 1 seconds for safety

            response.success = True
            response.message = "UV curing successful!"
        
        except PmRobotError as e:
            response.success = False
            response.message = f"Error in uv_curing_callback: {e}"
            self.logger.error(f"{response.message}")
            return response

        return response


    def set_uv_cart_positions(self, request:pm_msg_srv.SetUvSliderXPositions.Request, response:pm_msg_srv.SetUvSliderXPositions.Response):
        
        cmd = Float64MultiArray()
        cmd.data.append(request.uv_slider_front_position_mm*1e-3) # convert to m
        cmd.data.append(request.uv_slider_back_position_mm*1e-3) # convert to m 
        self.uv_cart_publisher.publish(cmd)

        self.logger.warning(f"Setting 'UV Cart Front' to '{request.uv_slider_front_position_mm} mm'")
        self.logger.warning(f"Setting 'UV Cart Back' to '{request.uv_slider_back_position_mm} mm'")
        response.success = True
        return response
    

    def move_uv_in_curing_position_service_callback(self, request:SetBool.Request, response:SetBool.Response):
        """Moves both UV LEDs in curing position"""
        try:
            response.success = self.pm_primitive_skills_utils.move_uv_in_curing_position(request)
            response.message = "UV LEDs moved in curing position!" 
        except PmRobotError as e:
            self.get_logger().error(f"Error moving UV LEDs in curing position: {e.message}")
            response.success = False
            response.message = e.message
        return response
    
    def dispense_at_points_callback(self, request: pm_msg_srv.DispenseAtPoints.Request, response:pm_msg_srv.DispenseAtPoints.Response):
        
        dispenser_prepared = False

        try:
            for dispense_point in request.dispense_points:
                dispense_point: pm_msg_msg.DispensePoint
                move_to_frame_request = pm_moveit_srv.MoveToFrame.Request()
                
                move_to_frame_request.target_frame = dispense_point.frame_name
                move_to_frame_request.execute_movement = True
                
                
                if not dispenser_prepared:
                    self.pm_primitive_skills_utils.prepare_dispenser(move_to_frame_request)
                    dispenser_prepared = True

                self.pm_primitive_skills_utils.dispense_at_frame(move_to_frame_request, 
                                                dispense_point.frame_name,
                                                time=dispense_point.time_ms,
                                                dispense_z_offset_mm=dispense_point.dispense_z_offset_mm)
                        
            self.pm_primitive_skills_utils.retract_dispenser()
            time.sleep(0.5)
            self.pm_primitive_skills_utils.close_protection()
            
            response.success = True

        except PmRobotError as e:
            self.get_logger().error(f"Error during dispensing at points: {e.message}")
            response.success = False
            response.message = e.message
        return response
    
    def dispense_test_point_callback(self, request: pm_msg_srv.DispenseTestPoints.Request, response:pm_msg_srv.DispenseTestPoints.Response):
        num = request.num_of_points
        
        dispenser_prepared = False
        
        try:
            for i in range(num):

                if not self.adhesive_test_point_grid_1.is_station_saturated():
                    current_station = self.adhesive_test_point_grid_1
                    self.get_logger().info(f"Dispensing test point {current_station.get_current_point()}/{current_station._max_points} at station 1!")
                elif not self.adhesive_test_point_grid_2.is_station_saturated():
                    self.get_logger().warn("Station 1 is saturated! Using station 2")
                    current_station = self.adhesive_test_point_grid_2
                    self.get_logger().info(f"Dispensing test point {current_station.get_current_point()}/{current_station._max_points} at station 2!")
                else:
                    self.get_logger().warn("Both stations are saturated!")
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
                    self.pm_primitive_skills_utils.prepare_dispenser(move_to_frame_request)
                    dispenser_prepared = True

                self.pm_primitive_skills_utils.dispense_at_frame(move_to_frame_request, 
                                        point_name="Test_Point_" + str(current_station.get_current_point()+1),
                                        time=request.time_ms,
                                        dispense_z_offset_mm=request.dispense_z_offset_mm)
                
                current_station.increment_current_point()

            time.sleep(1)
            self.pm_primitive_skills_utils.retract_dispenser()
            self.pm_primitive_skills_utils.close_protection()
            time.sleep(1)

        except PmRobotError as e:
            self.get_logger().error(f"Error during dispensing at test points: {e.message}")
            response.success = False

        return response
    

    def get_confocal_top_measurement_callback(self, request: GetValue.Request, response: GetValue.Response):

        try:
            _response: GetValue.Response = self.pm_primitive_skills_utils.get_confocal_top_mes()
            response = _response
        except PmRobotError as e:
            self.get_logger().error(f"Error getting confocal top measurement: {e.message}")
            response.success = False

        return response

    def get_confocal_bottom_measurement_callback(self, request: GetValue.Request, response: GetValue.Response):
        try:
            _response: GetValue.Response = self.pm_primitive_skills_utils.get_confocal_bottom_mes()
            response = _response
        except PmRobotError as e:
            self.get_logger().error(f"Error getting confocal bottom measurement: {e.message}")
            response.success = False
        return response

    def reset_test_station_callback(self, request: pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        self.adhesive_test_point_grid_1.reset_current_point()
        self.adhesive_test_point_grid_2.reset_current_point()
        response.success = True
        return response
    
    
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
