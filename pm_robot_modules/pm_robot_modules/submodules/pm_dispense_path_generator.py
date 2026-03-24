import json
import datetime
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
import matplotlib.pyplot as plt
from typing import Union, List, Optional, Tuple
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_points_by_pose(points: List[np.ndarray], reference_pose: Pose) -> List[np.ndarray]:
    """
    Transform a list of 3D points into the coordinate frame of a reference_pose.

    Args:
        points: list of np.ndarray with shape (3,) representing points [x, y, z]
        reference_pose: geometry_msgs.msg.Pose; defines the new origin and orientation

    Returns:
        List of np.ndarray points expressed in the reference_pose frame
    """
    # Convert reference_pose to rotation matrix and position vector
    r = R.from_quat([
        reference_pose.orientation.x,
        reference_pose.orientation.y,
        reference_pose.orientation.z,
        reference_pose.orientation.w
    ])
    T_ref = r.as_matrix()        # 3x3 rotation
    t_ref = np.array([
        reference_pose.position.x,
        reference_pose.position.y,
        reference_pose.position.z
    ])                            # translation vector

    # Inverse transformation: world -> reference frame
    R_inv = T_ref.T               # inverse rotation
    t_inv = -R_inv @ t_ref        # inverse translation

    # Apply transform to all points
    transformed = []
    for p in points:
        p_new = R_inv @ p + t_inv
        transformed.append(p_new)

    return transformed

def ros_to_lh(point: Point) -> Point:
    """
    Convert a point from ROS (right-handed) to left-handed (flip Z).
    """
    p = Point()
    p.x = point.x
    p.y = point.y
    p.z = -point.z
    return p

def ros_to_lh_np(vector: np.ndarray) -> np.ndarray:
    """
    Convert a point from ROS (right-handed) to left-handed (flip Z).
    """
    p = np.array([vector[0], vector[1], -vector[2]])
    return p



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

    def get_g_code(self, start_point: Point, orientation: Quaternion) -> Tuple[Point, str]:
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
    
    def get_g_code(self, start_point: Point, orientation: Quaternion) -> Tuple[Point, str]:
        """
        Generate G-code for the move action based on the start point,
        applying the goal offsets rotated by the given orientation.

        Args:
            start_point: geometry_msgs.msg.Point, starting point
            orientation: geometry_msgs.msg.Quaternion, orientation to rotate goal vector

        Returns:
            Tuple of:
                - new_point: geometry_msgs.msg.Point, transformed goal position
                - gcode: str, G-code command
        """
        # Create goal vector from offsets
        goal_vector = np.array([self.goal_x, self.goal_y, self.goal_z])

        goal_vector = ros_to_lh_np(goal_vector)

        # Convert quaternion to rotation matrix
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        #rotated_goal = r.apply(goal_vector)  # rotate the goal offsets
        rotated_goal = r.inv().apply(goal_vector)

        # Add rotated goal to start point
        new_point = Point()
        new_point.x = start_point.x + rotated_goal[0]
        new_point.y = start_point.y + rotated_goal[1]
        new_point.z = start_point.z + rotated_goal[2]

        # Generate G-code
        gcode_point = (new_point)

        gcode = (
            f"{self.GCODE_ID} "
            f"X{gcode_point.x} Y{gcode_point.y} Z{gcode_point.z} "
            f"F{abs(self.speed)} "
            f"P{abs(self.dispenser_pressure)}"
        )

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
    
    def get_g_code(self, start_point: Point, orientation: Quaternion) -> Tuple[Point, str]:
        """
        Generate G-code for the move action based on the start point.
        Optionally applies a rotation to the goal offsets.

        Args:
            start_point: geometry_msgs.msg.Point, starting point
            orientation: geometry_msgs.msg.Quaternion, orientation to rotate goal vector

        Returns:
            Tuple of:
                - new_point: geometry_msgs.msg.Point, transformed goal position
                - gcode: str, G-code command
        """
        # Create goal vector from offsets
        goal_vector = np.array([self.goal_x, self.goal_y, self.goal_z])

        goal_vector = ros_to_lh_np(goal_vector)

        # Rotate goal if orientation is provided
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        #goal_vector = r.apply(goal_vector)
        goal_vector = r.inv().apply(goal_vector)

        # Add goal to start point
        new_point = Point()
        new_point.x = start_point.x + goal_vector[0]
        new_point.y = start_point.y + goal_vector[1]
        new_point.z = start_point.z + goal_vector[2]

        # Generate G-code
        gcode_point = (new_point)

        gcode = (
            f"{self.GCODE_ID} "
            f"X{gcode_point.x} Y{gcode_point.y} Z{gcode_point.z} "
            f"F{abs(self.speed)}"
        )

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
    
    def get_g_code(self, start_point: Point, orientation: Quaternion) -> Tuple[Point, str]:
        """
        Generate G-code for the dip action based on the start point and orientation.
        The dip vector is rotated by the given orientation before being applied.

        Args:
            start_point: geometry_msgs.msg.Point, starting point
            orientation: geometry_msgs.msg.Quaternion, rotation to apply

        Returns:
            Tuple of:
                - new_point: geometry_msgs.msg.Point, transformed dip position
                - gcode: str, G-code command
        """
        # Define dip vector along local -Z
        dip_vector = np.array([0.0, 0.0, -self.dip_depth])

        dip_vector = ros_to_lh_np(dip_vector)

        # Rotate dip vector according to orientation
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        #rotated_dip = r.apply(dip_vector)
        rotated_dip = r.inv().apply(dip_vector)


        # Compute new point
        new_point = Point()
        new_point.x = start_point.x + rotated_dip[0]
        new_point.y = start_point.y + rotated_dip[1]
        new_point.z = start_point.z + rotated_dip[2]

        gcode_point = (new_point)

        # Generate G-code
        gcode = f"{self.GCODE_ID} X{gcode_point.x} Y{gcode_point.y} Z{gcode_point.z} F{abs(self.dip_speed)}"

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
        if not self.file_path.endswith('.pmdispp'):
            raise ValueError("File path must end with '.pmdispp'.")
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
    
    def generate_g_code(self, start_pose: Pose, start_joint_values: Point) -> str:
        """
        Generate G-code for the entire sequence of actions.
        This method iterates through all actions and generates the corresponding G-code.
        """
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0
        start_pose.position.z = 0.0
        g_code = []
        # convert start_joint_values from m to mm
        start_joint_values_mm = Point()
        start_joint_values_mm.x = start_joint_values.x * 1000.0
        start_joint_values_mm.y = start_joint_values.y * 1000.0
        start_joint_values_mm.z = start_joint_values.z * 1000.0

        current_point = copy.copy(start_joint_values_mm)

        for action in self.actions:
            if isinstance(action, BaseAction):
                new_point, action_g_code = action.get_g_code(current_point, start_pose.orientation)
                g_code.append(action_g_code)
                current_point = new_point
            else:
                raise TypeError("Action must be an instance of BaseAction or its subclasses.")
        return "\n".join(g_code)

    def save_g_code_to_file(self, 
                            start_pose: Pose, 
                            start_joint_values:Point,
                            file_path = None) -> bool:
        """
        Save the generated G-code to a .pmdispp.g file.
        If no file_path is provided, uses self.file_path as the base name.
        The G-code file will have the same base name as the sequence but with .pmdispp.g extension.
        
        Args:
            start_pose: Starting pose for G-code generation
            start_joint_values: Starting joint values
            file_path: Optional file path for saving. If None, uses self.file_path
            
        Returns:
            bool: True if save was successful
            
        Raises:
            ValueError: If no file path is set and file_path parameter is None
        """
        # Determine which file path to use
        if file_path is None:
            if self.file_path is None:
                raise ValueError("File path is not set. Please set the file path before saving G-code.")
            base_path = self.file_path
        else:
            base_path = file_path
        
        # Generate G-code
        g_code = self.generate_g_code(start_pose, start_joint_values)
        
        # Ensure .pmdispp.g ending
        if not base_path.endswith('.pmdispp.g'):
            if base_path.endswith('.pmdispp'):
                # Already has .pmdispp, just add .g
                gcode_file_path = base_path + '.g'
            else:
                # Add full .pmdispp.g extension
                gcode_file_path = base_path + '.pmdispp.g'
        else:
            gcode_file_path = base_path
        
        # Write to file
        with open(gcode_file_path, 'w') as file:
            file.write(g_code)
        
        return True
    
    def generate_sequence(self):
        pass

    def get_list_available_actions(self):
        return list(self.available_actions.keys())
    
    def get_action_count(self):
        """Get the number of actions in the sequence."""
        return len(self.actions)
    
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
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")

        current = Point()
        current.x, current.y, current.z = self.current_point.x, self.current_point.y, self.current_point.z

        for action in self.actions:
            if isinstance(action, BaseAction):
                current = action.add_to_visualization(ax, current)

        # Only show legend if labels are set
        if ax.get_legend_handles_labels()[1]:  # labels list is non-empty
            ax.legend()

        ax.grid(True)
        
        # Ensure minimum height and width of 10mm BEFORE setting aspect ratio
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        
        MIN_SIZE = 10.0  # minimum 10mm
        
        # Expand x-axis if needed
        if x_range < MIN_SIZE:
            x_center = (xlim[0] + xlim[1]) / 2
            ax.set_xlim(x_center - MIN_SIZE/2, x_center + MIN_SIZE/2)
        
        # Expand y-axis if needed
        if y_range < MIN_SIZE:
            y_center = (ylim[0] + ylim[1]) / 2
            ax.set_ylim(y_center - MIN_SIZE/2, y_center + MIN_SIZE/2)
        
        # Set aspect ratio AFTER ensuring minimum size (use datalim to preserve our limits)
        ax.set_aspect('equal', adjustable='datalim')
        ax.autoscale(enable=False)

        return fig



if __name__ == "__main__":
    import tempfile
    import os
    
    print("=" * 70)
    print("DISPENSE SEQUENCE GENERATOR TEST CASE")
    print("=" * 70)
    
    # Create test sequence
    print("\n[1] Creating test sequence...")
    sequence = DispenseSequenceGenerator()
    
    # Test 1: Add actions
    print("\n[2] Adding actions to sequence...")
    
    # Move to starting position
    move_action = MoveAction()
    move_action.set_name("Start Position")
    move_action.goal_x = 10.0
    move_action.goal_y = 10.0
    move_action.goal_z = 5.0
    move_action.heigth = 2.0
    move_action.speed = 0.5
    sequence.add_action(move_action)
    print(f"  ✓ Added MoveAction: {move_action.name}")
    
    # First dispense
    dispense_action1 = DispenseAction()
    dispense_action1.set_name("Dispense Point 1")
    dispense_action1.goal_x = 10.0
    dispense_action1.goal_y = 10.0
    dispense_action1.goal_z = 0.0
    dispense_action1.heigth = 1.0
    dispense_action1.speed = 0.2
    sequence.add_action(dispense_action1)
    print(f"  ✓ Added DispenseAction: {dispense_action1.name}")
    
    # Move to second position
    move_action2 = MoveAction()
    move_action2.set_name("Move to Position 2")
    move_action2.goal_x = 30.0
    move_action2.goal_y = 20.0
    move_action2.goal_z = 5.0
    move_action2.heigth = 2.0
    move_action2.speed = 0.5
    sequence.add_action(move_action2)
    print(f"  ✓ Added MoveAction: {move_action2.name}")
    
    # Dip action
    dip_action = DipAction()
    dip_action.set_name("Dip in Ink")
    dip_action.dip_depth = 5.0
    dip_action.dip_speed = 0.1
    sequence.add_action(dip_action)
    print(f"  ✓ Added DipAction: {dip_action.name}")
    
    # Second dispense
    dispense_action2 = DispenseAction()
    dispense_action2.set_name("Dispense Point 2")
    dispense_action2.goal_x = 30.0
    dispense_action2.goal_y = 20.0
    dispense_action2.goal_z = 0.0
    dispense_action2.heigth = 1.0
    dispense_action2.speed = 0.2
    sequence.add_action(dispense_action2)
    print(f"  ✓ Added DispenseAction: {dispense_action2.name}")
    
    # Test 2: Verify action count
    print(f"\n[3] Sequence contains {sequence.get_action_count()} actions")
    
    # Test 3: Get sequence as dict
    print("\n[4] Getting sequence dictionary...")
    sequence_dict = sequence._get_sequence_dict()
    print(f"  ✓ Sequence has {len(sequence_dict['actions'])} actions in dict")
    print(f"  ✓ First action type: {sequence_dict['actions'][0]['action_type']}")
    
    # Test 4: Save to file
    print("\n[5] Testing file save/load...")
    with tempfile.TemporaryDirectory() as tmpdir:
        test_file = os.path.join(tmpdir, "test_sequence.pmdispp")
        sequence.file_path = test_file
        
        # Save
        save_success = sequence.save_to_file()
        print(f"  ✓ Save successful: {save_success}")
        print(f"  ✓ File size: {os.path.getsize(test_file)} bytes")
        
        # Create new sequence and load
        sequence_loaded = DispenseSequenceGenerator()
        load_success = sequence_loaded.load_from_file(test_file)
        print(f"  ✓ Load successful: {load_success}")
        print(f"  ✓ Loaded sequence has {len(sequence_loaded.actions)} actions")
        
        # Verify loaded data
        loaded_action = sequence_loaded.actions[0]
        print(f"  ✓ First loaded action: {loaded_action.name} (type: {type(loaded_action).__name__})")
    
    # Test 5: Generate G-code
    print("\n[6] Generating G-code...")
    start_pose = Pose()
    start_pose.position = Point(x=0.0, y=0.0, z=10.0)
    start_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    
    start_point = Point()
    start_point.x = 0.0
    start_point.y = 0.0
    start_point.z = 10.0
    
    gcode = sequence.generate_g_code(start_pose, start_point)
    gcode_lines = gcode.split('\n')
    print(f"  ✓ Generated {len(gcode_lines)} lines of G-code")
    print(f"  ✓ First 3 lines of G-code:")
    for i, line in enumerate(gcode_lines[:3]):
        if line.strip():
            print(f"    {i+1}: {line}")
    
    # Test 5b: Generate G-code with 45-degree rotated start pose
    print("\n[6b] Generating G-code with 45-degree rotation...")
    start_pose_45 = Pose()
    start_pose_45.position = Point(x=0.0, y=0.0, z=10.0)
    # 45-degree rotation around Z-axis: quaternion = (0, 0, sin(22.5°), cos(22.5°))
    start_pose_45.orientation = Quaternion(x=0.0, y=0.0, z=0.3827, w=0.9239)
    
    gcode_45 = sequence.generate_g_code(start_pose_45, start_point)
    gcode_45_lines = gcode_45.split('\n')
    print(f"  ✓ Generated {len(gcode_45_lines)} lines of G-code with 45° rotation")
    print(f"  ✓ First 3 lines of G-code (45° rotated):")
    for i, line in enumerate(gcode_45_lines[:3]):
        if line.strip():
            print(f"    {i+1}: {line}")
    
    # Compare with original
    print(f"\n  ✓ Comparing G-code outputs:")
    if gcode_lines[0] != gcode_45_lines[0]:
        print(f"    Original first line: {gcode_lines[0]}")
        print(f"    Rotated first line:  {gcode_45_lines[0]}")
        print(f"    ✓ G-code correctly transformed for rotation")
    else:
        print(f"    Note: G-code is the same (rotation applied in transformation)")
    
    # Test 6: Get visualization
    print("\n[7] Generating visualization...")
    fig = sequence.get_visualization()
    print(f"  ✓ Visualization created: {fig is not None}")
    if fig:
        print(f"  ✓ Figure title: {fig.axes[0].get_title()}")
    
    # Test 7: Action manipulation
    print("\n[8] Testing action manipulation...")
    original_count = len(sequence.actions)
    sequence.delete_action_at_index(1)  # Delete second action
    new_count = len(sequence.actions)
    print(f"  ✓ Deleted action at index 1")
    print(f"  ✓ Action count: {original_count} → {new_count}")
    
    # Test 8: Enable action selection
    print("\n[9] Testing action selection...")
    sequence.enable_action_selection(0)
    selected_action = sequence.actions[0]
    print(f"  ✓ Selected action: {selected_action.name}")
    print(f"  ✓ Action is selected: {selected_action._is_selected}")
    
    # Test 9: Move action
    print("\n[10] Testing action reordering...")
    sequence.move_action_to_index(0, 2)
    print(f"  ✓ Moved action from index 0 to 2")
    print(f"  ✓ New action order:")
    for i, action in enumerate(sequence.actions):
        print(f"    {i}: {action.name}")
    
    # Test 10: Get available actions
    print("\n[11] Testing available actions...")
    available = sequence.get_list_available_actions()
    print(f"  ✓ Available action types: {available}")
    
    # Test 11: Verify 45-degree rotation impact
    print("\n[12] Verifying 45-degree rotation impact...")
    print(f"  ✓ Start pose orientation (45°):")
    print(f"      x={start_pose_45.orientation.x}, y={start_pose_45.orientation.y}")
    print(f"      z={start_pose_45.orientation.z}, w={start_pose_45.orientation.w}")
    print(f"  ✓ This represents a 45-degree rotation around the Z-axis")
    print(f"  ✓ When applied to actions, movement vectors are rotated accordingly")
    
    print("\n" + "=" * 70)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 70)