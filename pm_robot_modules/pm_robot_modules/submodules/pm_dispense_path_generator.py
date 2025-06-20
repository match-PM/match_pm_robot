import json
import datetime
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from typing import Union
from typing import Tuple  # Make sure this is imported
import copy

class BaseAction:
    def __init__(self):
        self.name = self.__class__.__name__  # Default name is the class name
        self._is_selected = False  # Flag to indicate if this action is selected

    def get_dict(self):
        dict_data = self.__dict__
        # Remove _is_selected from the dictionary if it exists
        return dict_data

    def _set_from_dict(self, data):
        for key, value in data.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise KeyError(f"'{key}' is not a valid attribute of {self.__class__.__name__}")

    def set_name(self, name:str):
        if isinstance(name, str):
            self.name = name
        else:
            raise TypeError("Name must be a string.")
        
    def update_current_point(self, point:Point):
        raise NotImplementedError("This method should be implemented in subclasses.")
        
    def add_to_visualization(self):
        raise NotImplementedError("This method should be implemented in subclasses.")

    def get_g_code(self, start_point: Point)-> Tuple[Point, str]:
        """
        Generate G-code for the action based on the start point.
        This method should be overridden in subclasses to provide specific G-code generation.
        """
        raise NotImplementedError("This method should be implemented in subclasses.")

class DispenseAction(BaseAction):
    GCODE_ID = "G10"
    LINE_COLOR = 'r'
    LINE_WIDTH = [3, 4]  # Default line width for unselected and

    def __init__(self):
        super().__init__()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.heigth = 0.0
        self.speed = 0.0
        self.dispenser_pressure = 0.0

    def test(self):
        print("Test method called in DispenseAction")
        return "Test successful"
    
    def update_current_point(self, point:Point):
        if isinstance(point, Point):
            point.x += self.goal_x
            point.y += self.goal_y
            point.z += self.goal_z
        else:
            raise TypeError("Point must be an instance of geometry_msgs.msg.Point")
        
    
    def add_to_visualization(self, ax, current_point: Point):
        # This method should implement the logic to add this action to a visualization
        # For example, it could plot the action on a matplotlib figure
        # draw line from the current point to the goal point
        new_point = Point()
        new_point.x = current_point.x + self.goal_x
        new_point.y = current_point.y + self.goal_y
        new_point.z = current_point.z + self.goal_z 
        
        if self._is_selected:
            linewidth = self.LINE_WIDTH[1]
        else:
            linewidth = self.LINE_WIDTH[0]

        ax.plot([current_point.x, new_point.x],
            [current_point.y, new_point.y],
            self.LINE_COLOR, label=self.name, linewidth=linewidth)
        
        # Optionally, mark the new point with a red dot
        #ax.plot(new_point.x, new_point.y, 'ro')

        return new_point  # Return the new point for further actions to use
    
    def get_g_code(self, start_point: Point) -> Tuple[Point, str]:
        """
        Generate G-code for the move action based on the start point.
        This method generates a G-code command to move to the specified goal position.
        """
        
        new_point = Point()
        new_point.x = start_point.x + self.goal_x
        new_point.y = start_point.y + self.goal_y
        new_point.z = start_point.z + self.goal_z 

        gcode = f"{self.GCODE_ID} X{new_point.x} Y{new_point.y} Z{new_point.z} F{abs(self.speed)} P{abs(self.dispenser_pressure)}"
        return new_point, gcode

class MoveAction(BaseAction):
    GCODE_ID = "G20"
    LINE_WIDTH = [1 ,2]
    LINE_COLOR = 'k'

    def __init__(self):
        super().__init__()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.heigth = 0.0
        self.speed = 0.0
        self.test = 0.0

    def update_current_point(self, point:Point):
        if isinstance(point, Point):
            point.x += self.goal_x
            point.y += self.goal_y
            point.z += self.goal_z

    def add_to_visualization(self, ax, current_point: Point):
        # This method should implement the logic to add this action to a visualization
        # For example, it could plot the action on a matplotlib figure
        # draw line from the current point to the goal point
        new_point = Point()
        new_point.x = current_point.x + self.goal_x
        new_point.y = current_point.y + self.goal_y
        new_point.z = current_point.z + self.goal_z 
        
        if self._is_selected:
            linewidth = self.LINE_WIDTH[1]
        else:
            linewidth = self.LINE_WIDTH[0]

        ax.plot([current_point.x, new_point.x],
            [current_point.y, new_point.y],
            self.LINE_COLOR, label=self.name, linewidth=linewidth)
        
        return new_point  # Return the new point for further actions to use
    
    def get_g_code(self, start_point: Point) -> Tuple[Point, str]:
        """
        Generate G-code for the move action based on the start point.
        This method generates a G-code command to move to the specified goal position.
        """
        
        new_point = Point()
        new_point.x = start_point.x + self.goal_x
        new_point.y = start_point.y + self.goal_y
        new_point.z = start_point.z + self.goal_z 

        gcode = f"{self.GCODE_ID} X{new_point.x} Y{new_point.y} Z{new_point.z} F{abs(self.speed)}"
        return new_point, gcode
        

class DipAction(BaseAction):
    GCODE_ID = "G30"
    LINE_WIDTH = [1 ,2]

    def __init__(self):
        super().__init__()
        self.dip_depth = 0.0
        self.dip_speed = 0.0


    def update_current_point(self, point:Point):
        pass

    def add_to_visualization(self, ax, current_point: Point):
        # This method should implement the logic to add this action to a visualization
        # For example, it could plot the action on a matplotlib figure
        # draw circle at the current point with radius dip_depth

        if self._is_selected:
            linewidth = self.LINE_WIDTH[1]
        else:
            linewidth = self.LINE_WIDTH[0]
    
        circle = plt.Circle((current_point.x, current_point.y), self.dip_depth,
                            color='g', fill=False, label=self.name, linewidth=linewidth)
        ax.add_artist(circle)

        return current_point  # Return the current point as no movement is made in this action
    
    def get_g_code(self, start_point: Point) -> Tuple[Point, str]:
        """
        Generate G-code for the dip action based on the start point.
        This method generates a G-code command to dip to the specified depth.
        """
        gcode = f"{self.GCODE_ID} Z{start_point.z - self.dip_depth} F{abs(self.dip_speed)}"
        return start_point, gcode


class DispenseSequenceGenerator:
    def __init__(self):
        self.actions = []
        self.available_actions = {
            'DispenseAction': DispenseAction,
            'MoveAction': MoveAction,
            'DipAction': DipAction
        }  # Dictionary to map action names to classes
        self.file_path = None
        self.current_point = Point()  # Current position of the robot

    def add_action(self, action):
        if isinstance(action, BaseAction):
            self.actions.append(action)
        else:
            raise TypeError("Action must be an instance of BaseAction or its subclasses.")

    def _get_sequence_dict(self):
        sequence_dict = {}
        action_list = []

        for action in self.actions:
            action:BaseAction
            action_dict = {}
            action_parameters = action.get_dict().copy()  # Get a copy of the action's parameters
            action_parameters.pop('_is_selected', None)  # Remove the _is_selected attribute if
            action_type = action.__class__.__name__
            action_dict['action_type'] = action_type
            action_dict['parameters'] = action_parameters
            action_list.append(action_dict)
        sequence_dict['actions'] = action_list
        return sequence_dict

    def save_to_file(self)->bool:
        if self.file_path is None:
            raise ValueError("File path is not set. Please set the file path before saving.")
        if not self.file_path.endswith('.json'):
            raise ValueError("File path must end with '.json'.")
        sequence_dict = self._get_sequence_dict()
        sequence_dict['save_timestamp'] = datetime.datetime.now().isoformat()
        with open(f"{self.file_path}", 'w') as file:
            json.dump(sequence_dict, file, indent=4)

        return True

    def disable_action_selction(self):
        for action in self.actions:
                action._is_selected = False

    def enable_action_selection(self, index: int):
        if 0 <= index < len(self.actions):
            self.disable_action_selction()
            self.actions[index]._is_selected = True
        else:
            raise IndexError("Index out of range for actions list.")
        
    def load_from_file(self, file_path)->bool:
        with open(f"{file_path}", 'r') as file:
            sequence_dict = json.load(file)
            self.actions = []
            for action_data in sequence_dict['actions']:
                action_type = action_data['action_type']
                parameters = action_data['parameters']

                if action_type == 'DispenseAction':
                    action = DispenseAction()
                elif action_type == 'MoveAction':
                    action = MoveAction()
                elif action_type == 'DipAction':
                    action = DipAction()
                else:
                    raise ValueError(f"Unknown action type: {action_type}")

                action._set_from_dict(parameters)
                self.add_action(action)
            print(self._get_sequence_dict())
                # action.set_name(action_data.get('name', ''))
        
        self.file_path = file_path
        return True
    
    def generate_g_code(self, start_joint_values: Point) -> str:
        """
        Generate G-code for the entire sequence of actions.
        This method iterates through all actions and generates the corresponding G-code.
        """
        g_code = []
        current_point = copy.copy(start_joint_values)
        for action in self.actions:
            if isinstance(action, BaseAction):
                new_point, action_g_code = action.get_g_code(current_point)
                g_code.append(action_g_code)
                current_point = new_point
            else:
                raise TypeError("Action must be an instance of BaseAction or its subclasses.")
        return "\n".join(g_code)

    def save_g_code_to_file(self, start_joint_values:Point) -> bool:
        """
        Save the generated G-code to a txt file.
        This method generates the G-code and writes it to the specified file path.
        """
        g_code = self.generate_g_code(start_joint_values)
        
        file_path_txt = self.file_path
        # ensure txt ending
        if not file_path_txt.endswith('.txt'):
            if file_path_txt.endswith('.json'):
                # strip
                file_path_txt = file_path_txt[:-5]
            file_path_txt += ".txt"

        with open(file_path_txt, 'w') as file:
            file.write(g_code)
        return True
    
    def generate_sequence(self):
        pass

    def get_list_available_actions(self):
        return list(self.available_actions.keys())
    
    def clear_actions(self):
        self.actions.clear()

    def delete_action_at_index(self, index):
        if 0 <= index < len(self.actions):
            del self.actions[index]
        else:
            raise IndexError("Index out of range for actions list.")
        
    def move_action_to_index(self, old_index, new_index):
        if 0 <= old_index < len(self.actions) and 0 <= new_index < len(self.actions):
            action = self.actions.pop(old_index)
            self.actions.insert(new_index, action)
            print(f"Moved action from index {old_index} to {new_index}")
        else:
            raise IndexError("Index out of range for actions list.")

    def get_visualization(self) -> plt.Figure:
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title("Dispense Sequence Visualization")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        current = Point()
        current.x, current.y, current.z = self.current_point.x, self.current_point.y, self.current_point.z

        for action in self.actions:
            if isinstance(action, BaseAction):
                current = action.add_to_visualization(ax, current)

        # Only show legend if labels are set
        if ax.get_legend_handles_labels()[1]:  # labels list is non-empty
            ax.legend()

        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')

        return fig


if __name__ == "__main__":
    pass