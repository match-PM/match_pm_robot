import rclpy
from rclpy.node import Node
import time

import yaml

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
from pm_robot_modules.submodules.pm_robot_config import PmRobotConfig
from pm_robot_modules.submodules.configuration_requirements import (
    RobotConfigurationState,
    read_configuration,
    resolve_configuration_file,
)

try:
    from smarpod_interfaces.srv import SetVirtualPivot
    from std_srvs.srv import Trigger
    from pm_robot_primitive_skills.py_modules.smarpod_virtual_controller import SmarpodVirtualController
    smarpod_missing_package = None
except ImportError as error:
    # ModuleNotFoundError: package not installed; ImportError: installed
    # smarpod_interfaces is too old to provide SetVirtualPivot.
    if (error.name or '').split('.')[0] not in (
            'smarpod_interfaces', 'controller_manager_msgs', 'std_srvs'):
        raise
    smarpod_missing_package = error.name if isinstance(error, ModuleNotFoundError) else str(error)
    SmarpodVirtualController = None

class PrimitiveSkillsNode(Node):
    
    def __init__(self):
        super().__init__('pm_robot_primitive_skills')

        self.pm_primitive_skills_utils = PrimitiveSkillsUtils(self)

        if not self.has_parameter('use_real_config'):
            self.declare_parameter('use_real_config', False)
        self.use_real_config = bool(
            self.get_parameter('use_real_config').value
        )
        self.pm_robot_config = PmRobotConfig(
            use_real_config=self.use_real_config
        )
        self.declare_parameter('loaded_pm_robot_configuration_yaml', '')
        loaded_yaml = self.get_parameter(
            'loaded_pm_robot_configuration_yaml'
        ).value
        loaded_config = yaml.safe_load(loaded_yaml) if loaded_yaml else None
        if loaded_config is not None and (
                not isinstance(loaded_config, dict) or not loaded_config):
            raise ValueError('The launch-time configuration must be a nonempty mapping.')
        self.configuration_state = RobotConfigurationState(
            self.pm_robot_config.get_active_bringup_config_path(), loaded_config
        )

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        sim_time = self.get_parameter('use_sim_time').value
        
        self.logger = self.get_logger()
        if loaded_config is None:
            self.logger.warning(
                'No launch-time robot configuration was supplied; configuration '
                'checks will fail until started with the bringup snapshot.'
            )
        
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
        if SmarpodVirtualController is not None:
            self.smarpod_switch_group = MutuallyExclusiveCallbackGroup()
            self.smarpod_virtual_controller = SmarpodVirtualController(self)
            self.activate_smarpod_virtual_controller_srv = self.create_service(
                SetVirtualPivot, self.get_name()+'/activate_smarpod_virtual_controller',
                self.activate_smarpod_virtual_controller_callback, callback_group=self.smarpod_switch_group)
            self.reset_smarpod_virtual_controller_srv = self.create_service(
                Trigger, self.get_name()+'/reset_smarpod_virtual_controller',
                self.reset_smarpod_virtual_controller_callback, callback_group=self.smarpod_switch_group)
        else:
            self.logger.warning(
                f"Optional package '{smarpod_missing_package}' is unavailable; "
                'SmarPod virtual controller services disabled')
        
        self.get_confocal_top_measurement_srv = self.create_service(GetValue, self.get_name()+'/get_confocal_top_measurement', self.get_confocal_top_measurement_callback)
        self.get_confocal_bottom_measurement_srv = self.create_service(GetValue, self.get_name()+'/get_confocal_bottom_measurement', self.get_confocal_bottom_measurement_callback)
        
        self.set_uv_cart_position_srv = self.create_service(pm_msg_srv.SetUvSliderXPositions, self.get_name()+'/set_uv_slider_manual_positions', self.set_uv_cart_positions, callback_group=self.callback_group_mu_ex)
        
        self.uv_cart_publisher = self.create_publisher(Float64MultiArray,"/pm_uv_cart_manual_controller/commands",10)

        self.set_gripper_2_jaws_position_srv = self.create_service(pm_msg_srv.Gripper2JawSetPosition, self.get_name()+'/set_gripper_2_jaws_position', self.set_gripper_2_jaws_position_callback, callback_group=self.callback_group_mu_ex)

        self.gripper_2_jaws_publisher = self.create_publisher(Float64MultiArray,"/pm_parallel_gripper_2_jaws_controller/commands",10)

        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.current_config_publisher = self.create_publisher(
            pm_msg_msg.PmRobotConfiguration,
            self.get_name() + '/current_pm_robot_configuration',
            config_qos,
        )
        self.config_callback_group = MutuallyExclusiveCallbackGroup()
        self.check_config_service = self.create_service(
            pm_msg_srv.CheckPmRobotConfiguration,
            self.get_name() + '/check_pm_robot_configuration',
            self.check_pm_robot_configuration_callback,
            callback_group=self.config_callback_group,
        )
        self.set_config_service = self.create_service(
            pm_msg_srv.SetPmRobotConfiguration,
            self.get_name() + '/set_pm_robot_configuration',
            self.set_pm_robot_configuration_callback,
            callback_group=self.config_callback_group,
        )
        self.change_tool_service = self.create_service(
            pm_msg_srv.ChangeTool,
            self.get_name() + '/change_tool',
            self.change_tool_callback,
            callback_group=self.config_callback_group,
        )
        self.current_config_timer = self.create_timer(
            5.0,
            self.publish_current_pm_robot_configuration,
            callback_group=self.callback_group_re,
        )
        self.publish_current_pm_robot_configuration()
        
        config_mode = 'real hardware' if self.use_real_config else 'simulation'
        self.logger.info(
            f"Primitive skills node started! Using sim time: {sim_time}; "
            f"bringup configuration mode: {config_mode}"
        )

    def publish_current_pm_robot_configuration(self):
        """Publish the immutable configuration captured by robot bringup."""
        try:
            current_config = self.configuration_state.loaded
            if current_config is None:
                return

            message = pm_msg_msg.PmRobotConfiguration()
            message.config_file_path = (
                self.pm_robot_config.get_active_bringup_config_path()
            )
            message.use_real_config = self.use_real_config
            message.config_yaml = yaml.safe_dump(
                current_config,
                default_flow_style=False,
                sort_keys=False,
            )

            self.current_config_publisher.publish(message)
        except Exception as error:
            self.logger.error(
                'Failed to publish the current PM robot configuration: '
                f'{error}'
            )

    @staticmethod
    def _count_configuration_properties(configuration):
        if isinstance(configuration, dict):
            return sum(
                PrimitiveSkillsNode._count_configuration_properties(value)
                for value in configuration.values()
            )
        return 1

    def check_pm_robot_configuration_callback(self, request, response):
        """Check requirements against the loaded robot and the current file."""
        try:
            requirement_path = resolve_configuration_file(
                request.rsap_path,
                request.file_path_override,
            )
            required_config = read_configuration(requirement_path)
            differences = self.configuration_state.check(required_config)
            if differences:
                response.success = False
                response.message = (
                    f"PM robot configuration does not match "
                    f"'{requirement_path}':\n- "
                    + '\n- '.join(differences)
                )
                return response

            checked_properties = self._count_configuration_properties(
                required_config
            )
            response.success = True
            response.message = (
                f"Loaded and on-disk configurations match '{requirement_path}' "
                f"({checked_properties} properties checked)."
            )
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.logger.error(response.message)

        return response

    def set_pm_robot_configuration_callback(self, request, response):
        """Merge requested fields into the bringup file for the next launch."""
        try:
            requirement_path = resolve_configuration_file(
                request.rsap_path, request.file_path_override
            )
            required_config = read_configuration(requirement_path)
            restart_required = self.configuration_state.update(required_config)
            response.success = True
            response.message = (
                f"Updated bringup configuration '{self.configuration_state.path}' "
                f"using '{requirement_path}'. "
                + ('Restart the robot to load the configuration.'
                   if restart_required else 'The robot already has this configuration loaded.')
            )
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.logger.error(response.message)
        return response

    def change_tool_callback(self, request, response):
        """Save a tool and attachment for the next launch without restarting."""
        try:
            tool_name = request.tool_name.strip()
            attachment_name = request.attachment_name.strip()
            # The launch supplies use_real_config for the robot this node serves.
            # Load a fresh copy to preserve other changes made to the file.
            config = PmRobotConfig(use_real_config=self.use_real_config)
            grippers = [
                config.tool._gripper_vacuum,
                config.tool._gripper_1_jaw,
                config.tool._gripper_2_jaw,
            ]
            selected = None
            available_tools = []
            for gripper in grippers:
                for name, attachments in gripper.get_available_tools():
                    available_tools.append(name)
                    if name == tool_name:
                        selected = gripper
                        available_attachments = attachments
            if selected is None:
                raise ValueError(
                    f"Tool '{tool_name}' not found in config. "
                    f"Available: {available_tools}"
                )
            if not attachment_name or attachment_name not in available_attachments:
                raise ValueError(
                    f"Invalid attachment '{attachment_name}' for tool '{tool_name}'. "
                    f"Available: {available_attachments}"
                )

            for gripper in grippers:
                gripper.deactivate()
            selected.activate()
            selected.set_current_tool(tool=tool_name, ext=attachment_name)
            config.save_config()
            response.success = True
            response.message = (
                f"Tool changed to '{tool_name}' with attachment '{attachment_name}'. "
                f"Config saved to '{config.get_active_bringup_config_path()}'. "
                'Restart the robot to load the configuration.'
            )
            self.logger.info(response.message)
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.logger.error(response.message)
        return response

    def activate_smarpod_virtual_controller_callback(self, request, response):
        response.success, response.message = self.smarpod_virtual_controller.activate(
            request.frame_name)
        if not response.success:
            self.get_logger().error(response.message)
        return response

    def reset_smarpod_virtual_controller_callback(self, request, response):
        response.success, response.message = self.smarpod_virtual_controller.reset()
        if not response.success:
            self.get_logger().error(response.message)
        return response
        

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

    def set_gripper_2_jaws_position_callback(self, request:pm_msg_srv.Gripper2JawSetPosition.Request, response:pm_msg_srv.Gripper2JawSetPosition.Response):
        """Sets the 2-jaw parallel gripper to the given position for both jaws."""
        try:
            cmd = Float64MultiArray()
            # Both jaws are set to the same value to keep the gripper symmetric.
            cmd.data.append(request.position)
            cmd.data.append(request.position)
            self.gripper_2_jaws_publisher.publish(cmd)

            self.logger.warn(f"Setting 'Tool_Parallel_Gripper_Jaw_1_Joint' and 'Tool_Parallel_Gripper_Jaw_2_Joint' to '{request.position} m'")
            response.success = True
        except Exception as e:
            self.logger.error(f"Error setting 2-jaw gripper position: {e}")
            response.success = False
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
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
