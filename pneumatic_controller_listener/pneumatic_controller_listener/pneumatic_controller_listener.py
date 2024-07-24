import rclpy
from rclpy.node import Node
import yaml
from yaml.loader import SafeLoader
from std_msgs.msg import String, Bool, Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from functools import partial
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, Parameter, ParameterType
import xml.etree.ElementTree as ET 
from collections import defaultdict
from example_interfaces.srv import SetBool

def etree_to_dict(t):
    d = {t.tag: {} if t.attrib else None}
    children = list(t)
    if children:
        dd = defaultdict(list)
        for dc in map(etree_to_dict, children):
            for k, v in dc.items():
                dd[k].append(v)
        d = {t.tag: {k:v[0] if len(v) == 1 else v for k, v in dd.items()}}
    if t.attrib:
        d[t.tag].update(('@' + k, v) for k, v in t.attrib.items())
    if t.text:
        text = t.text.strip()
        if children or t.attrib:
            if text:
              d[t.tag]['#text'] = text
        else:
            d[t.tag] = text
    return d

class PneumaticControllerListener(Node):

    def __init__(self):
        super().__init__('pm_pneumatic_controller_listener')
        self.robot_description_path = get_package_share_directory('pm_robot_description')
        self.controller_yaml = f"{self.robot_description_path}/config/pm_robot_control_real_HW.yaml"
        self.test_publisher = self.create_publisher(Float64MultiArray,"/pm_pneumatic_forward_controller/commands",10)

        self.logger = self.get_logger()
        self.pneumatic_controller_names = []
        self.forward_controller_names = []
        self.init_controller_names()
        #self.init_subscriber()

        self.bool= False
        self.robot_desciption = None
        self.lower_limits = []
        self.upper_limits = []
        self.state = []

        self.init_robot_description()
        self.extract_limits()     
        self.init_services()  
        # create a service
        #self.srv_flap = self.create_service(SetBool, '/pm_pneumatic_dummy/set_dispenser_flap', self.set_flap)
        #self.srv_dispenser = self.create_service(SetBool, '/pm_pneumatic_dummy/set_dispenser', self.set_dispenser)

    def init_services(self):
        for controller_name in self.forward_controller_names:
            srv = self.create_service(SetBool, f'/pm_pneumatic_dummy/set_{controller_name}', partial(self.set_srv, controller_name=controller_name))

    def set_srv(self, request:SetBool.Request, response:SetBool.Response, controller_name:str):
        has_changed, new_value = self.set_axis_forward(controller_name, request.data)

        if has_changed:
            cmd = Float64MultiArray()
            cmd.data = self.state
            self.test_publisher.publish(cmd)
            self.logger.error(f"Setting '{controller_name}' to '{new_value}', isForward=={request.data}!")
        
        response.success = True
        
        return response  
    
    def init_robot_description(self):
        cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        while not cli.wait_for_service(timeout_sec=10.0):
            self.logger.info("Waiting for 'robot_state_publisher' to start...")

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = cli.call_async(request)
        while not future.done():
            rclpy.spin_once(self)

        response:GetParameters.Response = future.result()
        self.test:ParameterValue = response.values[0]
        self.robot_desciption = self.test.string_value

        # Extract XML
        xml = ET.fromstring(self.robot_desciption)
        self.robot_description_dict:dict = etree_to_dict(xml)

    def extract_limits(self):
        for axis_name in self.forward_controller_names:
            lower, upper = self.extract_limit(axis_name)
            self.lower_limits.append(lower)
            self.upper_limits.append(upper)
            self.state.append(lower)
        self.logger.info(f"Lower limits {self.lower_limits}")
        self.logger.info(f"Upper limits {self.upper_limits}")
        
    def extract_limit(self, axis_name:str):
        for sub_dict in self.robot_description_dict["robot"]["joint"]:
            joint_name = sub_dict['@name'] 
            if joint_name == axis_name:
                limit = sub_dict['limit'] 
                limit_lower = float(limit['@lower'])
                limit_upper = float(limit['@upper'])
                return (limit_lower, limit_upper)
            
    def init_subscriber(self):
        for controller_name in self.pneumatic_controller_names:
            _topic = f'/pm_pneumatic_controller/{controller_name}/IsForward'
            _publisher = self.create_subscription(Bool,_topic, partial(self.callback, controller_name=controller_name), 10)

    def init_controller_names(self):
        with open(self.controller_yaml) as f:
            controller_config = yaml.safe_load(f)
        self.pneumatic_controller_names = controller_config["pm_pneumatic_controller"]["ros__parameters"]["cylinders"]
        self.forward_controller_names = controller_config["pm_pneumatic_forward_controller"]["ros__parameters"]["joints"]

        for pneumatic_controller_name in self.pneumatic_controller_names:
            self.logger.warn(pneumatic_controller_name)

        for forward_controller_name in self.forward_controller_names:
            self.logger.warn(forward_controller_name)


    def callback(self, msg:Bool, controller_name:str):
        has_changed, new_value = self.set_axis_forward(self.get_joint_for_controller(controller_name), msg.data)
        if has_changed:
            cmd = Float64MultiArray()
            cmd.data = self.state
            self.test_publisher.publish(cmd)
            self.logger.error(f"Setting '{self.get_joint_for_controller(controller_name)}' to '{new_value}', isForward=={msg.data}!")

    def get_joint_for_controller(self, controller_name:str):
        for index, name in enumerate(self.pneumatic_controller_names):
            if name == controller_name:
                return self.forward_controller_names[index]
    
    def set_axis_forward(self, target_axis:str, value: bool)->bool:
        for index, axis in enumerate(self.forward_controller_names):
            if axis == target_axis:
                if value:
                    if self.state[index] != self.get_lower_value_for_axis(target_axis):
                        self.state[index] = self.get_lower_value_for_axis(target_axis)
                        return True, self.get_lower_value_for_axis(target_axis)
                    else:
                        return False, None
                else:
                    if self.state[index] != self.get_upper_value_for_axis(target_axis):
                        self.state[index] = self.get_upper_value_for_axis(target_axis)
                        return True, self.get_upper_value_for_axis(target_axis)
                    else:
                        return False, None
                
    def get_lower_value_for_axis(self, target_axis:str)->float:
        for index, axis in enumerate(self.forward_controller_names):
            if axis == target_axis:
                return self.lower_limits[index]

    def get_upper_value_for_axis(self, target_axis:str)->float:
        for index, axis in enumerate(self.forward_controller_names):
            if axis == target_axis:
                return self.upper_limits[index]

    
def main(args=None):
    rclpy.init(args=args)

    pneumatic_controller_listener = PneumaticControllerListener()

    rclpy.spin(pneumatic_controller_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pneumatic_controller_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()