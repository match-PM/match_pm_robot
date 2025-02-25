from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import yaml
from geometry_msgs.msg import Vector3

class TestPointGrid:
    def __init__(self, ind_of_test_grid: int = 0):
        
        self.ind_of_test_grid = ind_of_test_grid
        
        self.path = get_package_share_directory('pm_robot_bringup')
        self.file_path = f'{self.path}/config/pm_robot_bringup_config.yaml'
    
        self.load_config()
        self._init_values()     

    def load_config(self):
        with open(self.file_path, 'r') as file:
            self.config_data = yaml.safe_load(file)
            
        self.config_data_station = self.config_data['adhesive_test_point_station']
        self.grid_data = self.config_data_station[self.ind_of_test_grid]
    
    def _init_values(self):
        self.num_x_points = self.grid_data['num_x_points']
        self.num_y_points = self.grid_data['num_y_points']
        self.grid_x_spacing = self.grid_data['grid_x_spacing']  # in mm
        self.grid_y_spacing = self.grid_data['grid_y_spacing'] # in mm
        self.current_point = self.grid_data['current_point']
        self.parent_frame = self.grid_data['parent_frame']
        self.frame_id = self.parent_frame['frame_id']
        self.offset_x = self.parent_frame['offset_x']   # in mm
        self.offset_y = self.parent_frame['offset_y']  # in mm
        self.offset_z = self.parent_frame['offset_z'] # in mm
        self.roll = self.parent_frame['roll']
        self.pitch = self.parent_frame['pitch']
        self.yaw = self.parent_frame['yaw']
        self._max_points = self.num_x_points * self.num_y_points
        
    def save_current_point_to_file(self):
        self.grid_data['current_point'] = self.current_point
        #self.config_data[self.ind_of_test_grid] = self.grid_data
        with open(self.file_path, 'w') as file:
            yaml.dump(self.config_data, file)
            
    def increment_current_point(self)->bool:
        if self.is_station_saturated():
            print("Station is saturated!")
            return False
        else:
            self.current_point += 1
            self.save_current_point_to_file()
            return True
        
    def get_current_point(self):
        return self.current_point
    
    def get_start_offset(self)->Vector3:
        offset = Vector3()
        offset.x = float(self.offset_x/1000)
        offset.y = float(self.offset_y/1000)
        offset.z = float(self.offset_z/1000)
        return offset
    
    def get_current_offset(self)->Vector3:
        # first x direction
        # get_x_indice
        x_indice = self.current_point % self.num_x_points
        # get_y_indice
        y_indice = self.current_point // self.num_x_points
        
        x_offset = x_indice * self.grid_x_spacing
        y_offset = y_indice * self.grid_y_spacing
        
        offset = Vector3()
        offset.x = float(x_offset/1000)
        offset.y = float(y_offset/1000)
        
        return offset
    
    def reset_current_point(self):
        self.current_point = 0
        self.save_current_point_to_file()
        
    def is_station_saturated(self):
        print("Current point: ", self.current_point)
        print("Max points: ", self._max_points)
        return self.current_point >= self._max_points
        
if __name__ == '__main__':
    test_point_grid = TestPointGrid()
    #test_point_grid.reset_current_point()
    print(test_point_grid.get_current_offset())
    test_point_grid.increment_current_point()


