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

class PrimitiveSkillsUtils():
    DISPENSER_TRAVEL_DISTANCE = 0.04
    DISPENSER_OFFSET_VALUE = 0.01
    DEFAULT_DISPENSE_HEIGHT = 0.5 # mm
    
    REAL_MODE = 0
    UNITY_MODE = 1
    GAZEBO_MODE = 2

    def __init__(self, node: Node):
        self.node = node

        self.logger = self.node.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()
    
        self._current_force_sensor_data = Float64MultiArray()
       
        # create clients
        self.client_force_sensor = self.node.create_subscription(Float64MultiArray, '/pm_sensor_controller/ForceSensor/Stream',self.force_sensor_callback, 10)
        self.client_open_protection_gaz = self.node.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Protection_Joint',callback_group = self.callback_group_re)
        self.client_dispenser_joint_gaz = self.node.create_client(SetBool, '/pm_pneumatic_dummy/set_1K_Dispenser_Joint',callback_group = self.callback_group_re)
        self.client_2k_dispenser_joint_gaz = self.node.create_client(SetBool, '/pm_pneumatic_dummy/set_2K_Dispenser_Joint',callback_group = self.callback_group_re)

        self.client_open_protection_real = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveBackward',callback_group = self.callback_group_re)
        self.client_close_protection_real = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Protection_Joint/MoveForward',callback_group = self.callback_group_re)

        self.client_retract_dispenser_real = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveBackward',callback_group = self.callback_group_re)
        self.client_extend_dispenser_real = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N1K_Dispenser_Joint/MoveForward',callback_group = self.callback_group_re)
        self.dispenser_2k_forward_client = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N2K_Dispenser_Joint/MoveForward',callback_group = self.callback_group_re)
        self.dispenser_2k_backward_client = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/N2K_Dispenser_Joint/MoveBackward',callback_group = self.callback_group_re)

        self.client_dispense_skill = self.node.create_client(Dispense, '/pm_opcua_skills_controller/Dispense',callback_group = self.callback_group_re)

        self.client_create_adhesive_viz_point = self.node.create_client(pm_msg_srv.CreateVizAdhesivePoint, '/pm_adhesive_displayer/add_point',callback_group = self.callback_group_re)
        self.client_move_robot_1k_dips = self.node.create_client(MoveToFrame, '/pm_moveit_server/move_1k_dispenser_to_frame',callback_group=self.callback_group_re)
        self.client_move_cam = self.node.create_client(MoveToFrame, '/pm_moveit_server/move_cam1_to_frame',callback_group=self.callback_group_re)

        self.client_get_confocal_bottom_measurement = self.node.create_client(GetValue, '/uepsilon_two_channel_controller/IFC2422/ch2/distance/srv',callback_group=self.callback_group_re)
        self.client_get_confocal_top_measurement = self.node.create_client(GetValue, '/uepsilon_two_channel_controller/IFC2422/ch1/distance/srv',callback_group=self.callback_group_re)
        
        self.client_uv_front_forward= self.node.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Front_Joint/MoveForward",callback_group = self.callback_group_re)
        self.client_uv_front_backward= self.node.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Front_Joint/MoveBackward",callback_group = self.callback_group_re)
        self.client_uv_back_forward= self.node.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Back_Joint/MoveForward",callback_group = self.callback_group_re)
        self.client_uv_back_backward= self.node.create_client(pm_msg_srv.EmptyWithSuccess, "/pm_pneumatic_controller/UV_LED_Back_Joint/MoveBackward",callback_group = self.callback_group_re)

        self.client_dips_1k_on = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
        self.client_dips_1k_off = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)

        self.client_uv_controller_on = self.node.create_client(pm_msg_srv.UVSetOnOff, '/pm_uv_controller/Hoenle_UV/SetOnOff',callback_group = self.callback_group_re)
        self.client_uv_controller_time = self.node.create_client(pm_msg_srv.UVSetTime, '/pm_uv_controller/Hoenle_UV/SetTime',callback_group = self.callback_group_re)
        self.client_uv_controller_intensity = self.node.create_client(pm_msg_srv.UVSetPower, '/pm_uv_controller/Hoenle_UV/SetPower',callback_group = self.callback_group_re)

    def force_sensor_callback(self, msg: Float64MultiArray):
        self._current_force_sensor_data = msg

    def get_current_force_sensor_data(self):
        
        if len(self._current_force_sensor_data.data) == 0:
            raise PmRobotError("No force sensor data received yet!")
        
        return self._current_force_sensor_data

    def move_dispenser_to_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request,
                                z_offset_m = 0.0)-> pm_moveit_srv.MoveToFrame.Response:
        """
        Moves the dispenser to the specified frame with an optional z offset.
        Args:
            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.
            z_offset_m (float, optional): The offset in meters to be added to the z-axis of the target frame. Defaults to 0.0.
        Returns:
            pm_moveit_srv.MoveToFrame.Response: The response from the move_to_frame service call.
        Raises:
            PmRobotError: If the move_to_frame service is not available or if the service call fails.
        """
        if not self.client_move_robot_1k_dips.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_move_robot_1k_dips.srv_name}' not available")

        req = move_to_frame_request

        req.translation.z += z_offset_m
        
        response:pm_moveit_srv.MoveToFrame.Response = self.client_move_robot_1k_dips.call(req)
        if not response.success:
            raise PmRobotError("Failed to move dispenser to frame!")
        return response
            
    def create_adhesive_viz_point(self, request: pm_msg_srv.CreateVizAdhesivePoint.Request)->pm_msg_srv.CreateVizAdhesivePoint.Response:
        if not self.client_create_adhesive_viz_point.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_create_adhesive_viz_point.srv_name}' not available")
        
        response:pm_msg_srv.CreateVizAdhesivePoint.Response = self.client_create_adhesive_viz_point.call(request)
        return response
    
    
    def move_uv_in_curing_position(self, request:SetBool.Request) -> bool:
        
        if not self.client_uv_front_forward.wait_for_service(timeout_sec=2.0) and not self.client_uv_front_backward.wait_for_service(timeout_sec=2.0) and not self.client_uv_back_forward.wait_for_service(timeout_sec=2.0) and not self.client_uv_back_backward.wait_for_service(timeout_sec=2.0):
            raise PmRobotError("One or more UV LED control services not available!")

        request_empty = pm_msg_srv.EmptyWithSuccess.Request()

        if request.data == True:
            success_front = self.client_uv_front_forward.call(request_empty)
            success_back = self.client_uv_back_forward.call(request_empty)
        else:
            success_front = self.client_uv_front_backward.call(request_empty)
            success_back = self.client_uv_back_backward.call(request_empty)

        self.node.get_logger().info(f"Service call result: success={success_front}")
        self.node.get_logger().info(f"Service call result: success={success_back}")
        
        if not success_front or not success_back:
            raise PmRobotError("Failed to move UV LEDs in curing position!")
        
        return True

    def open_protection(self):
        """
        Opens the protection for the dispenser. If Gazebo is running, it calls the Gazebo service to open the protection. If a real robot or Unity is running, it calls the respective service to open the protection.
        Raises:            PmRobotError: If the service for opening the protection is not available or if the service call fails.
        """
        if self.is_gazebo_running():
            if not self.client_open_protection_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_open_protection_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = True
            response:SetBool.Response = self.client_open_protection_gaz.call(req)
            
            if not response.success:
                raise PmRobotError("Failed to open protection in Gazebo!")
        
        else:
            if not self.client_open_protection_real.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_open_protection_real.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.client_open_protection_real.call(req)
            
        if not response.success:
            raise PmRobotError("Failed to open protection on real/unity robot!")
    
    def close_protection(self):
        """
        Closes the protection for the dispenser. If Gazebo is running, it calls the Gazebo service to close the protection. If a real robot or Unity is running, it calls the respective service to close the protection.
        Raises:            PmRobotError: If the service for closing the protection is not available or if the service call fails.
        """
        if self.is_gazebo_running():
            if not self.client_open_protection_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_open_protection_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = False
            response:SetBool.Response = self.client_open_protection_gaz.call(req)

        else:
            if not self.client_close_protection_real.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_close_protection_real.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.client_close_protection_real.call(req)

        if not response.success:        
            raise PmRobotError("Failed to close protection on real/unity robot!")
         
    def retract_dispenser(self):
        """
        Retracts the dispenser. If Gazebo is running, it calls the Gazebo service to retract the dispenser. If a real robot or Unity is running, it calls the respective service to retract the dispenser.
        Raises:            PmRobotError: If the service for retracting the dispenser is not available or if the service call fails.
        """

        if self.is_gazebo_running():
            if not self.client_dispenser_joint_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_dispenser_joint_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = True
            response:SetBool.Response = self.client_dispenser_joint_gaz.call(req)

        else:
            if not self.client_retract_dispenser_real.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_retract_dispenser_real.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.client_retract_dispenser_real.call(req)
            
        if not response.success:
            raise PmRobotError("Failed to retract dispenser on real/unity robot!")
            
    def extend_dispenser(self):
        """
        Extends the dispenser. If Gazebo is running, it calls the Gazebo service to extend the dispenser. If a real robot or Unity is running, it calls the respective service to extend the dispenser.
        Raises:            PmRobotError: If the service for extending the dispenser is not available or if the service call fails.
        """
        if self.is_gazebo_running():
            if not self.client_dispenser_joint_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_dispenser_joint_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = False
            response:SetBool.Response = self.client_dispenser_joint_gaz.call(req)
        else:
            if not self.client_extend_dispenser_real.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_extend_dispenser_real.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.client_extend_dispenser_real.call(req)

        if not response.success:
            raise PmRobotError("Failed to extend dispenser on real/unity robot!")
               

    def retract_2k_dispenser(self):
        """
        Retracts the dispenser. If Gazebo is running, it calls the Gazebo service to retract the 2k dispenser. If a real robot or Unity is running, it calls the respective service to retract the dispenser.
        Raises:            PmRobotError: If the service for retracting the dispenser is not available or if the service call fails.
        """

        if self.is_gazebo_running():
            if not self.client_2k_dispenser_joint_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_2k_dispenser_joint_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = True
            response:SetBool.Response = self.client_2k_dispenser_joint_gaz.call(req)

        else:
            if not self.dispenser_2k_backward_client.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.dispenser_2k_backward_client.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.dispenser_2k_backward_client.call(req)
            
        if not response.success:
            raise PmRobotError("Failed to retract dispenser on real/unity robot!")
    
    def extend_2k_dispenser(self):
        """
        Extends the dispenser. If Gazebo is running, it calls the Gazebo service to extend the 2k dispenser. If a real robot or Unity is running, it calls the respective service to extend the dispenser.
        Raises:            PmRobotError: If the service for extending the dispenser is not available or if the service call fails.
        """
        if self.is_gazebo_running():
            if not self.client_2k_dispenser_joint_gaz.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.client_2k_dispenser_joint_gaz.srv_name}' not available")
            
            req = SetBool.Request()
            req.data = False
            response:SetBool.Response = self.client_2k_dispenser_joint_gaz.call(req)
        else:
            if not self.dispenser_2k_forward_client.wait_for_service(timeout_sec=1.0):
                raise PmRobotError(f"Service '{self.dispenser_2k_forward_client.srv_name}' not available")
            
            req = pm_msg_srv.EmptyWithSuccess.Request()
            response:pm_msg_srv.EmptyWithSuccess.Response = self.dispenser_2k_forward_client.call(req)

        if not response.success:
            raise PmRobotError("Failed to extend dispenser on real/unity robot!")
    
    def prepare_dispenser(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request) -> bool:
        """
        Prepares the dispenser by moving it to a position above the target frame, opening the protection, and extending the dispenser.
        Args:            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.
        Raises:            PmRobotError: If any of the steps (moving to frame, opening protection, extending dispenser) fail.
        """

        move_to_frame_request_copy = copy.deepcopy(move_to_frame_request)
        
        move_to_frame_request_copy.translation.z += self.DISPENSER_TRAVEL_DISTANCE
        
        move_to_frame_request_copy.translation.z += self.DISPENSER_OFFSET_VALUE
        
        self.move_dispenser_to_frame(move_to_frame_request_copy)
                
        self.open_protection()
                
        self.extend_dispenser()
        
        time.sleep(0.5)
        
        return True
        
    
    def dispense(self, time:float):
        """
        Dispenses for the given time in seconds.
        Args:            time (float): Time in seconds to dispense.
        Returns:            bool: True if dispensing was successful, False otherwise.
        Raises:            PmRobotError: If the dispense service is not available or if the service call fails.
        """

        if self.is_unity_running():
            self.node.get_logger().warn("Unity is running, skipping dispense command!")
            return True
        
        if not self.client_dispense_skill.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_dispense_skill.srv_name}' is not available!")

        request = Dispense.Request()
        request.time = int(time)

        response:Dispense.Response = self.client_dispense_skill.call(request)

        if not response.success:
            raise PmRobotError("Failed to execute dispense skill!")
        
        return True

    def get_confocal_top_mes(self)-> GetValue.Response:
        """
        Gets the confocal top measurement and converts it to micrometers.
        Returns:            GetValue.Response: The response containing the confocal top measurement in micrometers.
        Raises:            PmRobotError: If the service for getting the confocal top measurement is not available or if the service call fails.
        """
        request_cl = GetValue.Request()

        if not self.client_get_confocal_top_measurement.wait_for_service(1):
            raise PmRobotError(f"Service '{self.client_get_confocal_top_measurement.srv_name}' is not available!")
        
        response:GetValue.Response = self.client_get_confocal_top_measurement.call(request=request_cl)

        # convert and units to um 
        if not self.is_unity_running():
            response.data = -1*(response.data-1.0)*1e3
        else:
            response.data = response.data

        return response

    def get_confocal_bottom_mes(self)-> GetValue.Response:
        """
        Gets the confocal bottom measurement and converts it to micrometers.
        Returns:            GetValue.Response: The response containing the confocal bottom measurement in micrometers.
        Raises:            PmRobotError: If the service for getting the confocal bottom measurement is not available or if the service call fails.
        """
        if not self.client_get_confocal_bottom_measurement.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_get_confocal_bottom_measurement.srv_name}' is not available!")

        # Create a new request (or use input request if applicable)
        request_cl = GetValue.Request()

        # Call service asynchronously
        response:GetValue.Response = self.client_get_confocal_bottom_measurement.call(request_cl)

        response.success = response.success
        if not self.is_unity_running():
            response.data = -1*(response.data - 3.0) * 1e3  # convert to micrometers
        else:
            response.data = response.data

        return response
    
    def dispense_at_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request, 
                          point_name: str,
                          time:float = 0.5,
                          dispense_z_offset_mm: float = DEFAULT_DISPENSE_HEIGHT
                          ) -> bool:
        """
        Dispenses at the specified frame with the given time and z offset.
        Args:            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.            time (float, optional): Time in seconds to dispense. Defaults to 0.5.            dispense_z_offset_mm (float, optional): Z offset in millimeters for dispensing. Defaults to DEFAULT_DISPENSE_HEIGHT.
        Raises:            PmRobotError: If the move_dispenser_to_frame or dispense methods fail, or if the create_adhesive_viz_point service is not available or fails.    
        """
        
        # dispense
        if time <=0:
            raise PmRobotError(f"Dispense time ({time}) must be greater than 0!")

        # check flap open not needed
        # check dispenser extended not needed
        move_to_frame_request.translation.z += float(dispense_z_offset_mm*1e-3)
        move_to_frame_request.translation.z += self.DISPENSER_OFFSET_VALUE
        
        # move to top position
        self.move_dispenser_to_frame(move_to_frame_request)
        
        move_to_frame_request.translation.z -= self.DISPENSER_OFFSET_VALUE
        
        # move to dispenser position
        self.move_dispenser_to_frame(move_to_frame_request)
                
        self.dispense(time=time)

        create_adhesive_viz_point_request = pm_msg_srv.CreateVizAdhesivePoint.Request()
        create_adhesive_viz_point_request.point.parent_frame = move_to_frame_request.target_frame
        
        create_adhesive_viz_point_request.point.hight = dispense_z_offset_mm # in mm
        create_adhesive_viz_point_request.point.diameter = 1.0 # in mm
        create_adhesive_viz_point_request.point.name = point_name
        self.create_adhesive_viz_point(create_adhesive_viz_point_request)

        move_to_frame_request.translation.z += self.DISPENSER_OFFSET_VALUE
        
        self.move_dispenser_to_frame(move_to_frame_request)
        
        return True
    
    def is_gazebo_running(self):
        """Check if the Gazebo node is active."""
        node_names = self.node.get_node_names()
        if 'gazebo' in node_names:
            return True
        return False
    
    
    def is_unity_running(self)->bool:
        """Check if the Unity node is active."""
        node_names = self.node.get_node_names()
        if 'ROS2UnityCam1Publisher' in node_names:
            return True
        return False
    

    def get_mode(self)->int:
        """Get the current mode of the robot."""
        """
        Returns:
        0 - REAL_MODE
        1 - UNITY_MODE
        2 - GAZEBO_MODE
        """

        if self.is_gazebo_running():
            return self.GAZEBO_MODE
        elif self.is_unity_running():
            return self.UNITY_MODE
        else:
            return self.REAL_MODE

if __name__ == '__main__':
    pass
