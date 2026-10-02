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
from tf2_ros import Buffer, TransformListener

class PrimitiveSkillsUtils():
    DISPENSER_TRAVEL_DISTANCE = 0.04
    DISPENSER_OFFSET_VALUE = 0.01
    DEFAULT_DISPENSE_HEIGHT = 0.5 # mm

    # Stroke of the '2K_Dispenser_Joint' (see pm_robot.xacro), the 2K equivalent of
    # DISPENSER_TRAVEL_DISTANCE.
    DISPENSER_2K_TRAVEL_DISTANCE = 0.05
    # The moveit server has no dedicated move group for the 2K dispenser. The 2K TCP is
    # part of the Z_Axis chain just like the cam1 TCP, so the cam1 move group (pure XYZ)
    # is used and the constant offset between both TCPs is compensated in the request.
    DISPENSER_2K_TCP_FRAME = "2K_Dispenser_TCP"
    CAM1_TCP_FRAME = "Cam1_Toolhead_TCP"

    # Reference point of the adhesive dot visualization: a dispense of
    # ADHESIVE_DOT_REF_TIME_MS leaves a dot of ADHESIVE_DOT_REF_DIAMETER_MM. Other times are
    # scaled from this (see adhesive_dot_diameter_from_time) and clamped so that a very short
    # or a very long dispense still produces a dot that is visible but not absurd.
    ADHESIVE_DOT_REF_DIAMETER_MM = 4.0
    ADHESIVE_DOT_REF_TIME_MS = 500.0
    ADHESIVE_DOT_MIN_DIAMETER_MM = 0.4
    ADHESIVE_DOT_MAX_DIAMETER_MM = 10.0
    
    REAL_MODE = 0
    UNITY_MODE = 1
    GAZEBO_MODE = 2

    def __init__(self, node: Node):
        self.node = node

        self.logger = self.node.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()

        # TF buffer used to resolve the orientation of dispense target frames relative
        # to 'world', so that dispenser approach offsets can be applied along world Z.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

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

        self.client_move_calibration_target_forward = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveForward')
        self.client_move_calibration_target_backward = self.node.create_client(EmptyWithSuccess, '/pm_pneumatic_controller/Camera_Calibration_Platelet_Joint/MoveBackward')
    

        self.client_dips_1k_on = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
        self.client_dips_1k_off = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)

        # Valve of the 2K glue cartridge. Opening/closing it is what 'G50 A1'/'G50 A0'
        # does in the path based 2K dispensing.
        self.client_dips_2k_on = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Glue_2K_Doseur/Pressure',callback_group = self.callback_group_re)
        self.client_dips_2k_off = self.node.create_client(pm_msg_srv.EmptyWithSuccess,'/pm_nozzle_controller/Glue_2K_Doseur/TurnOff',callback_group = self.callback_group_re)

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
            
    def move_2k_dispenser_to_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request,
                                   z_offset_m = 0.0)-> pm_moveit_srv.MoveToFrame.Response:
        """
        Moves the 2K dispenser TCP to the specified frame with an optional z offset.

        There is no dedicated moveit service for the 2K dispenser. The cam1 move group is
        used instead (same XYZ axes), and the constant offset between the cam1 TCP and the
        2K dispenser TCP is compensated in the request.

        The endeffector override of the moveit server is deliberately NOT used for this:
        for the cam1 move group the override only replaces the position of the target pose
        and leaves its orientation undefined, which makes the requested `translation` (the
        approach and dispense offsets) be applied along arbitrary axes.

        Args:
            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.
            z_offset_m (float, optional): The offset in meters to be added to the z-axis of the target frame. Defaults to 0.0.
        Returns:
            pm_moveit_srv.MoveToFrame.Response: The response from the move_to_frame service call.
        Raises:
            PmRobotError: If the move_to_frame service is not available or if the service call fails.
        """
        if not self.client_move_cam.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_move_cam.srv_name}' not available")

        # The TCP compensation must not end up in the caller's request: the same request
        # is reused for the following moves of a dispense sequence, which would otherwise
        # apply the compensation again for every move.
        req = copy.deepcopy(move_to_frame_request)

        self.add_world_z_offset(req, z_offset_m)

        # Both TCPs are rigidly attached to the Z axis. Moving the cam1 TCP by the
        # (cam1 -> 2K) vector in the opposite direction puts the 2K TCP on the target.
        d_x, d_y, d_z = self._get_world_offset_between_frames(self.CAM1_TCP_FRAME,
                                                              self.DISPENSER_2K_TCP_FRAME)
        self.add_world_offset(req, -d_x, -d_y, -d_z)

        response:pm_moveit_srv.MoveToFrame.Response = self.client_move_cam.call(req)
        if not response.success:
            raise PmRobotError(f"Failed to move 2K dispenser to frame '{req.target_frame}': {response.message}")
        return response

    def _get_world_offset_between_frames(self, from_frame: str, to_frame: str):
        """
        Returns the vector from `from_frame` to `to_frame`, expressed in world axes.

        Args:            from_frame (str): Name of the frame the vector starts at.            to_frame (str): Name of the frame the vector points to.
        Returns:            tuple[float, float, float]: (dx, dy, dz) in meters along the world axes.
        Raises:            PmRobotError: If one of the frames can not be looked up.
        """
        try:
            transform_from = self.tf_buffer.lookup_transform('world', from_frame,
                                                             rclpy.time.Time(),
                                                             rclpy.duration.Duration(seconds=1.0))
            transform_to = self.tf_buffer.lookup_transform('world', to_frame,
                                                           rclpy.time.Time(),
                                                           rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            raise PmRobotError(f"Could not look up transform between '{from_frame}' and "
                               f"'{to_frame}' in 'world': {str(e)}")

        return (transform_to.transform.translation.x - transform_from.transform.translation.x,
                transform_to.transform.translation.y - transform_from.transform.translation.y,
                transform_to.transform.translation.z - transform_from.transform.translation.z)

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
    
    def add_world_z_offset(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request, dz: float):
        """
        Adds a vertical offset of `dz` meters along the world Z axis to the request translation.

        See `add_world_offset` for why the offset cannot simply be added to translation.z.

        Args:
            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request to modify in place.
            dz (float): Offset in meters along world +Z (positive = upwards).
        """
        self.add_world_offset(move_to_frame_request, 0.0, 0.0, dz)

    def add_world_offset(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request,
                         dx: float, dy: float, dz: float):
        """
        Adds an offset of (dx, dy, dz) meters along the world axes to the request translation.

        The moveit server applies `translation` in the target frame's local coordinates
        (the translation is post-multiplied onto the target frame pose). If the target
        frame's axes are not aligned with world (the frame is tilted), simply adding the
        offset to the translation would move the robot along the tilted local axes instead
        of the world axes, so the dispenser would not end up above the object.

        To compensate, the world offset vector is expressed in the target frame's local
        coordinates using the frame's orientation relative to 'world', and the resulting
        components are added to the request translation.

        Args:
            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request to modify in place.
            dx (float): Offset in meters along world +X.
            dy (float): Offset in meters along world +Y.
            dz (float): Offset in meters along world +Z (positive = upwards).
        """
        try:
            transform = self.tf_buffer.lookup_transform('world',
                                                        move_to_frame_request.target_frame,
                                                        rclpy.time.Time(),
                                                        rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            raise PmRobotError(f"Could not look up transform of frame "
                               f"'{move_to_frame_request.target_frame}' in 'world': {str(e)}")

        # Quaternion of the target frame expressed in world (world_R_frame).
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w

        # Express the world offset vector in the target frame's local coordinates:
        # v_local = R^T * v_world, i.e. the components of v_world along the columns of R.
        move_to_frame_request.translation.x += (dx * (1.0 - 2.0 * (y * y + z * z))
                                                + dy * 2.0 * (x * y + z * w)
                                                + dz * 2.0 * (x * z - y * w))
        move_to_frame_request.translation.y += (dx * 2.0 * (x * y - z * w)
                                                + dy * (1.0 - 2.0 * (x * x + z * z))
                                                + dz * 2.0 * (y * z + x * w))
        move_to_frame_request.translation.z += (dx * 2.0 * (x * z + y * w)
                                                + dy * 2.0 * (y * z - x * w)
                                                + dz * (1.0 - 2.0 * (x * x + y * y)))

    def prepare_dispenser(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request) -> bool:
        """
        Prepares the dispenser by moving it to a position above the target frame, opening the protection, and extending the dispenser.
        Args:            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.
        Raises:            PmRobotError: If any of the steps (moving to frame, opening protection, extending dispenser) fail.
        """

        move_to_frame_request_copy = copy.deepcopy(move_to_frame_request)
        
        move_to_frame_request_copy.execute_movement = True

        # Approach from above along world Z (independent of the target frame orientation).
        self.add_world_z_offset(move_to_frame_request_copy,
                                self.DISPENSER_TRAVEL_DISTANCE + self.DISPENSER_OFFSET_VALUE)

        self.move_dispenser_to_frame(move_to_frame_request_copy)
                
        self.open_protection()
                
        self.extend_dispenser()
        
        time.sleep(0.5)
        
        return True
        
    
    def prepare_2k_dispenser(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request) -> bool:
        """
        Prepares the 2K dispenser by moving it to a position above the target frame and extending it.

        Same logic as `prepare_dispenser`, but the 2K dispenser has no protection lid, so
        only the pneumatic cylinder ('2K_Dispenser_Joint') is moved forward.

        Args:            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.
        Raises:            PmRobotError: If any of the steps (moving to frame, extending dispenser) fail.
        """

        move_to_frame_request_copy = copy.deepcopy(move_to_frame_request)

        move_to_frame_request_copy.execute_movement = True

        # Approach from above along world Z (independent of the target frame orientation).
        self.add_world_z_offset(move_to_frame_request_copy,
                                self.DISPENSER_2K_TRAVEL_DISTANCE + self.DISPENSER_OFFSET_VALUE)

        self.move_2k_dispenser_to_frame(move_to_frame_request_copy)

        self.extend_2k_dispenser()

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
            #self.logger.error(f"DEBUG RESPONSE: top real {response.data} um")
        else:
            response.data = response.data
            #self.logger.error(f"DEBUG RESPONSE: top unity {response.data} um")

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

        #self.logger.error(f"DEBUG RESPONSE: init {response.data} um")

        result_class = GetValue.Response()
        result_class.success = response.success
        
        unity_active = self.is_unity_running()

        #self.logger.error(f"DEBUG UNITY ACTIVE: {unity_active}")

        if not unity_active:
            result_class.data = -1*(response.data - 3.0) * 1e3  # convert to micrometers
        else:
            result_class.data = response.data

        #self.logger.error(f"DEBUG RESPONSE: {response.data} um")

        return result_class
    
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
        # Offsets are applied along world Z so the dispenser stays above the object
        # even if the target frame is tilted.
        self.add_world_z_offset(move_to_frame_request,
                                float(dispense_z_offset_mm*1e-3) + self.DISPENSER_OFFSET_VALUE)

        # move to top position
        self.move_dispenser_to_frame(move_to_frame_request)

        self.add_world_z_offset(move_to_frame_request, -self.DISPENSER_OFFSET_VALUE)

        # move to dispenser position
        self.move_dispenser_to_frame(move_to_frame_request)
                
        self.dispense(time=time)

        create_adhesive_viz_point_request = pm_msg_srv.CreateVizAdhesivePoint.Request()
        create_adhesive_viz_point_request.point.parent_frame = move_to_frame_request.target_frame
        
        create_adhesive_viz_point_request.point.hight = dispense_z_offset_mm # in mm
        create_adhesive_viz_point_request.point.diameter = 1.0 # in mm
        create_adhesive_viz_point_request.point.name = point_name
        self.create_adhesive_viz_point(create_adhesive_viz_point_request)

        self.add_world_z_offset(move_to_frame_request, self.DISPENSER_OFFSET_VALUE)

        self.move_dispenser_to_frame(move_to_frame_request)
        
        return True
    
    def dispense_2k(self, time_ms:float):
        """
        Dispenses adhesive with the 2K dispenser for the given time in milliseconds.

        The 2K dispenser has no time controlled dispense skill. Instead the glue valve is
        opened and closed again after `time_ms`, which is exactly what the 'G50 A1' /
        'G50 A0' activation commands do in the path based 2K dispensing. Because the axes
        do not move while dispensing at a point, the xyz axis controller does not have to
        be deactivated (handed over to the machine controller) for this.

        Args:            time_ms (float): Time in milliseconds to keep the glue valve open.
        Returns:            bool: True if dispensing was successful.
        Raises:            PmRobotError: If the time is invalid, the valve services are not available or if a service call fails.
        """

        if time_ms <= 0:
            raise PmRobotError(f"Dispense time ({time_ms}) must be greater than 0!")

        # Neither Gazebo nor Unity provide the nozzle controller, so only the dwell is simulated.
        if self.get_mode() != self.REAL_MODE:
            self.node.get_logger().warn("Simulation is running, skipping 2K dispense command!")
            time.sleep(time_ms * 1e-3)
            return True

        if not self.client_dips_2k_on.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_dips_2k_on.srv_name}' is not available!")

        if not self.client_dips_2k_off.wait_for_service(timeout_sec=1.0):
            raise PmRobotError(f"Service '{self.client_dips_2k_off.srv_name}' is not available!")

        request = pm_msg_srv.EmptyWithSuccess.Request()

        response_on:pm_msg_srv.EmptyWithSuccess.Response = self.client_dips_2k_on.call(request)

        try:
            if not response_on.success:
                raise PmRobotError("Failed to open the 2K glue valve!")

            time.sleep(time_ms * 1e-3)

        finally:
            # Always close the valve again, also if the dwell was interrupted.
            response_off:pm_msg_srv.EmptyWithSuccess.Response = self.client_dips_2k_off.call(request)

        if not response_off.success:
            raise PmRobotError("Failed to close the 2K glue valve!")

        return True

    def adhesive_dot_diameter_from_time(self, time_ms: float) -> float:
        """
        Diameter (mm) of the visualized adhesive dot for a given dispense time.

        The dispensed volume grows linearly with the time the glue valve is open
        (V = Q * t), and a dot of volume V has a diameter proportional to V**(1/3), so the
        dot diameter grows with the cube root of the dispense time. The result is anchored
        at ADHESIVE_DOT_REF_DIAMETER_MM for ADHESIVE_DOT_REF_TIME_MS and clamped to
        [ADHESIVE_DOT_MIN_DIAMETER_MM, ADHESIVE_DOT_MAX_DIAMETER_MM].

        This is purely a visualization value (RViz marker / Unity adhesive dot), it does not
        describe the real dot on the component.

        Args:            time_ms (float): Time in milliseconds the glue valve is open.
        Returns:            float: Dot diameter in millimeters.
        """

        if time_ms <= 0:
            return self.ADHESIVE_DOT_MIN_DIAMETER_MM

        diameter = self.ADHESIVE_DOT_REF_DIAMETER_MM * (time_ms / self.ADHESIVE_DOT_REF_TIME_MS) ** (1.0 / 3.0)

        return float(min(max(diameter, self.ADHESIVE_DOT_MIN_DIAMETER_MM),
                         self.ADHESIVE_DOT_MAX_DIAMETER_MM))

    def dispense_2k_at_frame(self, move_to_frame_request: pm_moveit_srv.MoveToFrame.Request,
                             point_name: str,
                             time_ms:float = 500.0,
                             dispense_z_offset_mm: float = DEFAULT_DISPENSE_HEIGHT
                             ) -> bool:
        """
        Dispenses with the 2K dispenser at the specified frame with the given time and z offset.

        Same sequence as `dispense_at_frame`, but with the 2K dispenser TCP and the 2K glue valve.

        Args:            move_to_frame_request (pm_moveit_srv.MoveToFrame.Request): The request containing the target frame and movement parameters.            point_name (str): Name of the created adhesive visualization point.            time_ms (float, optional): Time in milliseconds to dispense. Defaults to 500.0.            dispense_z_offset_mm (float, optional): Z offset in millimeters for dispensing. Defaults to DEFAULT_DISPENSE_HEIGHT.
        Raises:            PmRobotError: If the move_2k_dispenser_to_frame or dispense_2k methods fail, or if the create_adhesive_viz_point service is not available or fails.
        """

        if time_ms <= 0:
            raise PmRobotError(f"Dispense time ({time_ms}) must be greater than 0!")

        # The dispenser is expected to be extended already (see prepare_2k_dispenser).
        # Offsets are applied along world Z so the dispenser stays above the object
        # even if the target frame is tilted.
        self.add_world_z_offset(move_to_frame_request,
                                float(dispense_z_offset_mm*1e-3) + self.DISPENSER_OFFSET_VALUE)

        # move to top position
        self.move_2k_dispenser_to_frame(move_to_frame_request)

        self.add_world_z_offset(move_to_frame_request, -self.DISPENSER_OFFSET_VALUE)

        # move to dispenser position
        self.move_2k_dispenser_to_frame(move_to_frame_request)

        self.dispense_2k(time_ms=time_ms)

        create_adhesive_viz_point_request = pm_msg_srv.CreateVizAdhesivePoint.Request()
        create_adhesive_viz_point_request.point.parent_frame = move_to_frame_request.target_frame

        create_adhesive_viz_point_request.point.hight = dispense_z_offset_mm # in mm
        # The longer the valve was open, the more adhesive is on the component — so the
        # visualized dot grows with the dispense time instead of being a fixed 1 mm.
        create_adhesive_viz_point_request.point.diameter = self.adhesive_dot_diameter_from_time(time_ms)
        create_adhesive_viz_point_request.point.name = point_name
        self.logger.info(f"2K dispense at '{point_name}': {time_ms} ms -> "
                         f"adhesive dot of {create_adhesive_viz_point_request.point.diameter:.3f} mm diameter")
        create_adhesive_viz_point_request.point.endeffector_frame = self.DISPENSER_2K_TCP_FRAME
        self.create_adhesive_viz_point(create_adhesive_viz_point_request)

        self.add_world_z_offset(move_to_frame_request, self.DISPENSER_OFFSET_VALUE)

        self.move_2k_dispenser_to_frame(move_to_frame_request)

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
