import sys
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import copy

class ParallelGripperConfig():
    TOOL_GRIPPER_1_JAW_IDENT = 'pm_robot_tool_parallel_gripper_1_jaw'
    TOOL_GRIPPER_2_JAW_IDENT = 'pm_robot_tool_parallel_gripper_2_jaws'
    
    def __init__(self, config_key, config_value) -> None:
        self._config_key = config_key
        self._config_value = config_value
        self._current_tool: str = self._config_value['use_tool']
        self._current_tool_attachement: str = self._config_value['use_jaw_type']
        self._tool_active: bool = self._config_value['use_paralell_gripper']
        
        if config_key == self.TOOL_GRIPPER_1_JAW_IDENT:
            self.data_file_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/parallel_gripper_1_jaw.yaml'
        else:
            self.data_file_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/parallel_gripper_2_jaw.yaml'
        
        self._available_tools = []
        _available_tools = self._config_value['available_tools']
        for tool in _available_tools:
            _available_tool_attachements = []
            
            for jaw in tool['availabe_jaws']:
                _available_tool_attachements.append(jaw)
            self._available_tools.append((tool['tool_name'], _available_tool_attachements)) 

    def set_current_tool(self, tool:str = None, ext:str = None):
        if tool is not None:
            self._current_tool = tool
            self._config_value['use_tool'] = tool
        if ext is not None:
            self._current_tool_attachement = ext
            self._config_value['use_jaw_type'] = ext

    def activate(self):
        self._tool_active = True
        self._config_value['use_paralell_gripper'] = True

    def deactivate(self):
        self._tool_active = False
        self._config_value['use_paralell_gripper'] = False

    def get_activate_status(self):
        return self._tool_active
    
    def get_config(self):
        return self._config_value
    
    def get_first_extension_for_current_tool(self)->str:
        return self._available_tools[0][1][0]

    def get_current_extension_list(self)->list[str]:
        for tool in self._available_tools:
            if tool[0] == self._current_tool:
                return tool[1]
        return []
    
    def get_current_tool(self)->str:
        return self._current_tool
    
    def get_current_tool_attachment(self)->str:
        return self._current_tool_attachement
    
    def get_available_tools(self)->list[str]:
        return self._available_tools
    
    def get_available_tool_attachments(self)->list[str]:
        for tool in self._available_tools:
            if tool[0] == self._current_tool:
                return tool[1]
        return []
    
    def get_calibration_frame_dict_file_name(self)->str:
        print(self.data_file_path)
        try:
            with open(self.data_file_path, 'r') as file:
                
                data = yaml.safe_load(file)
                return data[self._current_tool][self._current_tool_attachement]['calibration_file_name']
        except FileNotFoundError:
            return None
        return self.data_file_path
    
    def reload_config(self):
        self._tool_active = self._config_value['use_paralell_gripper']
        self._current_tool = self._config_value['use_tool']
        self._current_tool_attachement = self._config_value['use_jaw_type']
        _available_tools = self._config_value['available_tools']
        self._available_tools = []
        for tool in _available_tools:
            _available_tool_attachements = []
            for jaw in tool['availabe_jaws']:
                _available_tool_attachements.append(jaw)
            self._available_tools.append((tool['tool_name'], _available_tool_attachements))
            
            
class DispenserTipConfig():
    def __init__(self, config_value:dict) -> None:
        self._config_value = config_value
        self._current_dispenser_tip = config_value['use_dispenser_tip']
        self._available_dispenser_tips:list[str] = config_value['availabe_dispenser_tips']
    
    def set_currrent_dispenser_tip(self, tip:str):
        self._current_dispenser_tip = tip
        self._config_value['use_dispenser_tip'] = tip

    def get_config(self):
        return self._config_value
    
    def get_available_dispenser_tips(self)->list[str]:
        return self._available_dispenser_tips
    
    def get_current_dispenser_tip(self)->str:
        return self._current_dispenser_tip
    
    def reload_config(self):
        self._current_dispenser_tip = self._config_value['use_dispenser_tip']
        self._available_dispenser_tips:list[str] = self._config_value['availabe_dispenser_tips']

class GonioConfig():
    WITH_GONIO_LEFT = 'with_Gonio_Left'
    WITH_GONIO_RIGHT = 'with_Gonio_Right'
    WITH_SMARPOD_STATION = 'with_smarpod_station'
    GONIO_LEFT = 'pm_robot_gonio_left'
    GONIO_RIGHT = 'pm_robot_gonio_right'
    SMARPOD_STATION = 'pm_smparpod_station'
            
        
        
    # def __init__(self, config_2_key:str, config_value:dict) -> None:
    #     self._config_value = config_value
    #     self._config_2_key = config_2_key
    #     self._gonio_active = config_value[config_2_key]
    #     self._current_chuck = config_value['use_chuck']
    #     self._available_chucks:list[str] = config_value['availabe_chucks']
    
    def __init__(self, type_station: str, full_config_dict:dict) -> None:
        self._config_dict = full_config_dict
        
        self.key_type = type_station
        match type_station:
            case self.GONIO_LEFT:
                self.key_2 = self.WITH_GONIO_LEFT
            case self.GONIO_RIGHT:
                self.key_2 = self.WITH_GONIO_RIGHT
            case self.SMARPOD_STATION:
                self.key_2 = self.WITH_SMARPOD_STATION
        self.reload_config()
    
    def set_current_chuck(self, chuck:str):
        self._current_chuck = chuck
        self._config_dict[self.key_type]['use_chuck'] = chuck

    def activate(self):
        self._gonio_active = True
        self._config_dict[self.key_type][self.key_2] = True

    def deactivate(self):
        self._gonio_active = False
        self._config_dict[self.key_type][self.key_2] = False
    
    def get_available_chucks(self)->list[str]:
        return self._available_chucks
    
    def get_current_chuck(self)->str:
        return self._current_chuck
    
    def get_activate_status(self)->bool:    
        return self._gonio_active
    
    def reload_config(self):
        self._gonio_active = self._config_dict[self.key_type][self.key_2]
        self._current_chuck = self._config_dict[self.key_type]['use_chuck']
        self._available_chucks:list[str] = self._config_dict[self.key_type]['availabe_chucks']
    
    
class VacuumGripperConfig():
    TOOL_VACUUM_IDENT = 'pm_robot_vacuum_tools'

    def __init__(self, config_value) -> None:
        self._config_value = config_value
        self._current_tool = self._config_value['use_tool']
        self._current_tool_attachement = self._config_value['use_tip']
        self._tool_active = self._config_value['use_vacuum_tool']
        self._data_file_path = get_package_share_directory('pm_robot_description') + '/urdf/urdf_configs/schunk_vacuum_tools.yaml'

        self._available_tools = []
        _available_tools = self._config_value['availabe_tools']
        for tool in _available_tools:
            _available_tool_tips = []
            for jaw in tool['availabe_tips']:
                _available_tool_tips.append(jaw)
            self._available_tools.append((tool['tool_name'], _available_tool_tips)) 

    def set_current_tool(self, tool:str = None, ext:str = None):
        if tool is not None:
            self._current_tool = tool
            self._config_value['use_tool'] = tool
        if ext is not None:
            self._current_tool_attachement = ext
            self._config_value['use_tip'] = ext
    
    def activate(self):
        self._tool_active = True
        self._config_value['use_vacuum_tool'] = True

    def deactivate(self):
        self._tool_active = False
        self._config_value['use_vacuum_tool'] = False

    def get_activate_status(self):
        return self._tool_active
    
    def get_config(self):
        return self._config_value
    
    def get_first_extension_for_current_tool(self)->str:
        return self._available_tools[0][1][0]
    
    def get_current_extension_list(self)->list[str]:
        for tool in self._available_tools:
            if tool[0] == self._current_tool:
                return tool[1]
        return []
    
    def get_current_tool(self)->str:
        return self._current_tool
    
    def get_current_tool_attachment(self)->str:
        return self._current_tool_attachement
    
    def get_available_tools(self)->list[str]:
        return self._available_tools
    
    def get_calibration_frame_dict_file_name(self)->str:
        try:
            with open(self._data_file_path, 'r') as file:
                data = yaml.safe_load(file)
                return data[self._current_tool][self._current_tool_attachement]['calibration_file_name']
        except FileNotFoundError:
            print(f"File not found: {self._data_file_path}")
            return None
        
    def reload_config(self):
        self._tool_active = self._config_value['use_vacuum_tool']
        self._current_tool = self._config_value['use_tool']
        self._current_tool_attachement = self._config_value['use_tip']
        _available_tools = self._config_value['availabe_tools']
        self._available_tools = []
        for tool in _available_tools:
            _available_tool_tips = []
            for jaw in tool['availabe_tips']:
                _available_tool_tips.append(jaw)
            self._available_tools.append((tool['tool_name'], _available_tool_tips))
                
    
class PmRobotToolsConfig:    
    def __init__(self, config_data:dict):
        self._config_data = config_data
        self._gripper_vacuum = VacuumGripperConfig(self._config_data['pm_robot_tools'][VacuumGripperConfig.TOOL_VACUUM_IDENT])
        self._gripper_1_jaw = ParallelGripperConfig(config_key=ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT,
                                                    config_value=self._config_data['pm_robot_tools'][ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT])
        
        self._gripper_2_jaw = ParallelGripperConfig(config_key=ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT,
                                                    config_value = self._config_data['pm_robot_tools'][ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT])
        
    def get_active_tool_name(self):
        if self._gripper_vacuum.get_activate_status():
            tool_name = self._gripper_vacuum.get_current_tool()
            tool_attachement = self._gripper_vacuum.get_current_tool_attachment()
            return f'VacuumGripper active: Tool: {tool_name}, Tip: {tool_attachement}'
        elif self._gripper_1_jaw.get_activate_status():
            tool_name = self._gripper_1_jaw.get_current_tool()
            tool_attachement = self._gripper_1_jaw.get_current_tool_attachment()
            return f'Gripper 1 Jaw active: Tool: {tool_name}, Jaw: {tool_attachement}'
        elif self._gripper_2_jaw.get_activate_status():
            tool_name = self._gripper_2_jaw.get_current_tool()
            tool_attachement = self._gripper_2_jaw.get_current_tool_attachment()
            return f'Gripper 2 Jaw active: Tool: {tool_name}, Jaw: {tool_attachement}'
        else:
            raise RuntimeError("No tool is activated. Please activate a tool before using it.")
        return None
    
    def get_active_tool_type(self):
        if self._gripper_vacuum.get_activate_status():
            return VacuumGripperConfig.TOOL_VACUUM_IDENT
        elif self._gripper_1_jaw.get_activate_status():
            return ParallelGripperConfig.TOOL_GRIPPER_1_JAW_IDENT
        elif self._gripper_2_jaw.get_activate_status():
            return ParallelGripperConfig.TOOL_GRIPPER_2_JAW_IDENT
        else:
            raise RuntimeError("No tool is activated. Please activate a tool before using it.")
        return None
    
    def get_tool(self)-> VacuumGripperConfig | ParallelGripperConfig:
        if self._gripper_vacuum.get_activate_status():
            return self._gripper_vacuum
        elif self._gripper_1_jaw.get_activate_status():
            return self._gripper_1_jaw
        elif self._gripper_2_jaw.get_activate_status():
            return self._gripper_2_jaw
        else:
            raise RuntimeError("No tool is activated. Please activate a tool before using it.")
        
    def reload_config(self):
        self._gripper_vacuum.reload_config()
        self._gripper_1_jaw.reload_config()
        self._gripper_2_jaw.reload_config()
            
class PmRobotConfig:
    """
    This class is used to configure the robot.
    """

    def __init__(self):

        try:
            self.path = get_package_share_directory('pm_robot_bringup')
        except PackageNotFoundError as e:
            raise RuntimeError(f"Package 'pm_robot_bringup' not found. Please make sure it is installed.")
        
        self._config_data = {}
        self._file_path = f'{self.path}/config/pm_robot_bringup_config.yaml'
        self._load_config()
        self.tool = PmRobotToolsConfig(self._config_data)
        
        self.gonio_left = GonioConfig(type_station=GonioConfig.GONIO_LEFT,
                                      full_config_dict= self._config_data)
        
        self.gonio_right = GonioConfig(type_station=GonioConfig.GONIO_RIGHT,
                                        full_config_dict= self._config_data)
        
        self.dispenser_1k = DispenserTipConfig(self._config_data['pm_robot_1K_dispenser_tip'])
        
        self.smarpod_station = GonioConfig(type_station=GonioConfig.SMARPOD_STATION,
                                           full_config_dict= self._config_data)
        
    def _load_config(self):
        with open(self._file_path, 'r') as file:
            self._config_data = yaml.safe_load(file)
        
    def get_config_data(self):
        return copy.deepcopy(self._config_data)
    
    def save_config(self):
        with open(self._file_path, 'w') as file:
            yaml.dump(self._config_data, file, default_flow_style=False)
            
    def reload_config(self):
        self._load_config()
        self.tool.reload_config()
        self.gonio_left.reload_config()
        self.gonio_right.reload_config()
        self.dispenser_1k.reload_config()
        self.smarpod_station.reload_config()
        


    
if __name__ == "__main__":
    config = PmRobotConfig()
    config.tool.get_active_tool_name()
    print(config.tool.get_active_tool_name())
    