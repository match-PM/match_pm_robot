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
import time

class PrimitiveSkillsNode(Node):

    def __init__(self):
        super().__init__('pm_robot_primitive_skills')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        sim_time = self.get_parameter('use_sim_time').value
        
        self.srv = self.create_service(pm_msg_srv.DispenseForTime, self.get_name()+'/dispense_1K', self.dispense_callback)
        self.logger = self.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()
        
        self.adhesive_test_point_grid_1 = TestPointGrid(ind_of_test_grid=0)
        self.adhesive_test_point_grid_2 = TestPointGrid(ind_of_test_grid=1)
        
        self.client_dips_1k_on = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
        self.client_dips_1k_off = self.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)
        
        self.move_robot_tool_client = self.create_client(MoveToFrame, '/pm_moveit_server/move_1k_dispenser_to_frame',callback_group=self.callback_group_re)

        self.dispense_test_point_srv = self.create_service(pm_msg_srv.EmptyWithSuccess, self.get_name()+'/dispense_test_point', self.dispense_test_point_callback)

        self.reset_test_station_srv = self.create_service(pm_msg_srv.EmptyWithSuccess, self.get_name()+'/reset_test_station', self.reset_test_station_callback)
        self.logger.info("Primitive skills node started!")
        
        self.open_protection_srv = self.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint',callback_group = self.callback_group_re)

        self.dispenser_joint_srv = self.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Joint',callback_group = self.callback_group_re)
        
        self.create_adhesive_viz_point_srv = self.create_client(pm_msg_srv.CreateVizAdhesivePoint, '/adhesive_display_node/add_point',callback_group = self.callback_group_re)

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
        if not self.open_protection_srv.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint' not available")
            return False
        
        req = SetBool.Request()
        req.data = True
        response:SetBool.Response = self.open_protection_srv.call(req)
        return response.success
    
    def close_protection(self):
        if not self.open_protection_srv.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint' not available")
            return False
        
        req = SetBool.Request()
        req.data = False
        response:SetBool.Response = self.open_protection_srv.call(req)
        return response.success
    
    def retract_dispenser(self):
        if not self.dispenser_joint_srv.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Joint' not available")
            return False
        
        req = SetBool.Request()
        req.data = True
        response:SetBool.Response = self.dispenser_joint_srv.call(req)
        return response.success
    
    def extend_dispenser(self):
        if not self.dispenser_joint_srv.wait_for_service(timeout_sec=1.0):
            self.logger.error("Service '/pm_pneumatic_dummy/set_1K_Dispenser_Joint' not available")
            return False
        
        req = SetBool.Request()
        req.data = False
        response:SetBool.Response = self.dispenser_joint_srv.call(req)
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

    def dispense_test_point_callback(self, request: pm_msg_srv.EmptyWithSuccess.Request, response:pm_msg_srv.EmptyWithSuccess.Response):
        num = 5
        
        success_open_protection = self.open_protection()
        success_extend_dispenser = self.extend_dispenser()
        time.sleep(1)
        
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
            
            success = self.dispense_at_frame(move_to_frame_request)
            response.success = success
            
            if success:
                current_station.increment_current_point()

            else:
                self.logger.error("Dispensing at frame failed!")
                return response
        time.sleep(1)
        success_retract_dispenser = self.retract_dispenser()
        success_close_protection = self.close_protection()
        
        return response
    
    def dispense_at_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request)->bool:
        offset_value = 0.01
        
        move_to_frame_request.translation.z += float(0.001)
        move_to_frame_request.translation.z += offset_value

        success = self.move_dispenser_to_frame(move_to_frame_request)
        
        if not success:
            return False
        
        move_to_frame_request.translation.z -= offset_value
        
        success = self.move_dispenser_to_frame(move_to_frame_request)
        
        if not success:
            return False
        
        create_adhesive_viz_point_request = pm_msg_srv.CreateVizAdhesivePoint.Request()
        create_adhesive_viz_point_request.point.parent_frame = move_to_frame_request.target_frame
        
        create_adhesive_viz_point_request.point.hight = 1.0 # in mm
        create_adhesive_viz_point_request.point.diameter = 1.0 # in mm
        success_create_viz_point = self.create_adhesive_viz_point(create_adhesive_viz_point_request)

        move_to_frame_request.translation.z += offset_value
        
        success = self.move_dispenser_to_frame(move_to_frame_request)

        return success
    
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
