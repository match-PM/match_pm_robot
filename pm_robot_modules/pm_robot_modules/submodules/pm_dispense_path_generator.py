import json
import datetime
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class BaseAction:
    def __init__(self):
        self.name = self.__class__.__name__  # Default name is the class name

    def get_dict(self):
        return self.__dict__

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

class DispenseAction(BaseAction):
    GCODE_ID = "G10"

    def __init__(self):
        super().__init__()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.heigth = 0.0
        self.speed = 0.0

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
        new_point = Point()
        new_point.x = current_point.x + self.goal_x
        new_point.y = current_point.y + self.goal_y
        new_point.z = current_point.z + self.goal_z

        # Draw line from current to new
        ax.plot([current_point.x, new_point.x],
                [current_point.y, new_point.y],
                'r-', label=self.name)

        # Optional: mark the endpoint
        ax.plot(new_point.x, new_point.y, 'ro')

        return new_point

class MoveAction(BaseAction):
    GCODE_ID = "G20"

    def __init__(self):
        super().__init__()
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.heigth = 0.0
        self.speed = 0.0

    def update_current_point(self, point:Point):
        if isinstance(point, Point):
            point += self.goal_x
            point += self.goal_y
            point += self.goal_z
            raise TypeError("Point must be an instance of geometry_msgs.msg.Point")

    def add_to_visualization(self, ax, current_point: Point):
        # This method should implement the logic to add this action to a visualization
        # For example, it could plot the action on a matplotlib figure
        # draw line from the current point to the goal point
        new_point = Point()
        new_point.x = current_point.x + self.goal_x
        new_point.y = current_point.y + self.goal_y
        new_point.z = current_point.z + self.goal_z 
        ax.plot([current_point.x, new_point.x],
                [current_point.y, new_point.y],
                'b-', label=self.name)
        
        return new_point  # Return the new point for further actions to use
        

class DipAction(BaseAction):
    GCODE_ID = "G30"

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
        circle = plt.Circle((current_point.x, current_point.y), self.dip_depth,
                            color='g', fill=False, label=self.name)
        ax.add_artist(circle)

        return current_point  # Return the current point as no movement is made in this action


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
            action_parameters = action.get_dict()
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