
import sys
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6 import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QFrame,
    QHBoxLayout,
    QVBoxLayout,
    QMainWindow,
    QListWidget,
    QListWidgetItem,
    QDoubleSpinBox,
    QWidget,
    QVBoxLayout,
    QPushButton,
    QCheckBox,
    QLineEdit,
    QComboBox,
    QTextEdit,
    QLabel,
    QSlider,
    QSpinBox,
    QFontDialog,
    QFileDialog,
    QMenu,
    QScrollArea,
)
import json
import datetime
from PyQt6.QtGui import QAction
from PyQt6.QtCore import Qt
from functools import partial
import os

from pm_robot_modules.submodules.pm_dispense_path_generator import (
    DispenseSequenceGenerator,
    DispenseAction,
    MoveAction,
    DipAction,
    BaseAction)

class DispenserBuilderMainApp(QMainWindow):
    def __init__(self, initial_pipeline_file: str = None):
        super().__init__()
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        self.setWindowTitle("Dispenser Path Builder App")

        main_layout = QVBoxLayout()
        self.main_widget = DispenserBuilderWidget(initial_pipeline_file=initial_pipeline_file)
        main_layout.addWidget(self.main_widget)

        central_widget.setLayout(main_layout)
        self.setGeometry(100, 100, 1000, 800)

class DispenserBuilderWidget(QWidget):
    """
    Embeddable widget for building and editing dispense sequences.
    
    Signals:
        - file_loaded(str): Emitted when a file is successfully loaded
        - file_saved(str): Emitted when the sequence is saved
        - action_added(str): Emitted when an action is added  
        - action_removed(int): Emitted when an action is removed
        - action_modified(): Emitted when the current action is modified
        - sequence_changed(): Emitted when the sequence changes
    """
    
    # Define signals for external integration
    file_loaded = pyqtSignal(str)  # Emit when file is loaded with filename
    file_saved = pyqtSignal(str)   # Emit when file is saved with filepath
    action_added = pyqtSignal(str)  # Emit when action is added with action name
    action_removed = pyqtSignal(int)  # Emit when action is removed with index
    action_modified = pyqtSignal()  # Emit when action parameters are modified
    sequence_changed = pyqtSignal()  # Emit when sequence is modified
    
    def __init__(self, initial_pipeline_file: str = None):
        super().__init__()
        self.action_parameter_widgets = (
            []
        )  # this is a list of widgets that represent all the parameter of a vision function; it is used to display widgets when a vision function is klicked

        self.dispenser_action_list = DispenseSequenceGenerator()
        #self.dispenser_action_list.load_from_file("dispense_sequence.json")

        self.initUI()
        self.save_as = False  # This is a helper bool used for saving files

        # Load an initial file if given in startup.
        if initial_pipeline_file is not None:
            self.open_process_file(initial_pipeline_file)

    def initUI(self):
        """Initialize the UI with a better layout"""
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # ============================================================================
        # TOP TOOLBAR SECTION
        # ============================================================================
        toolbar_layout = QHBoxLayout()
        
        new_button = QPushButton('New')
        new_button.setMaximumWidth(70)
        new_button.clicked.connect(self.create_new_file)
        toolbar_layout.addWidget(new_button)
        
        open_button = QPushButton('Open')
        open_button.setMaximumWidth(70)
        open_button.clicked.connect(self.open_process_file)
        toolbar_layout.addWidget(open_button)
        
        save_button = QPushButton('Save')
        save_button.setMaximumWidth(70)
        save_button.clicked.connect(self.save_file)
        toolbar_layout.addWidget(save_button)
        
        save_as_button = QPushButton('Save As')
        save_as_button.setMaximumWidth(80)
        save_as_button.clicked.connect(self.save_file_as)
        toolbar_layout.addWidget(save_as_button)
        
        toolbar_layout.addSpacing(15)
        
        self.pipeline_name_widget = QLabel("Not saved yet")
        self.pipeline_name_widget.setStyleSheet("font-weight: bold; color: #333;")
        toolbar_layout.addWidget(self.pipeline_name_widget)
        toolbar_layout.addStretch()
        
        main_layout.addLayout(toolbar_layout)
        
        # ============================================================================
        # MAIN CONTENT AREA (horizontal split)
        # ============================================================================
        content_layout = QHBoxLayout()
        content_layout.setSpacing(10)
        
        # ============================================================================
        # LEFT PANEL - Action List and Controls
        # ============================================================================
        left_panel = QVBoxLayout()
        left_panel.setSpacing(5)
        
        left_panel.addWidget(QLabel("Actions:"))
        
        self.checkbox_list = ReorderableCheckBoxListWidget()
        self.checkbox_list.itemClicked.connect(self.action_in_list_clicked)
        self.checkbox_list.CustDragSig.connect(self.on_drop)
        self.checkbox_list.itemChanged.connect(self.parameter_change_callback)
        self.checkbox_list.setMinimumWidth(180)
        left_panel.addWidget(self.checkbox_list)
        
        # Action control buttons
        action_button_layout = QHBoxLayout()
        action_button_layout.setSpacing(3)
        
        add_button = QPushButton('Add')
        add_button.clicked.connect(self.show_vision_action_menu)
        action_button_layout.addWidget(add_button)
        
        delete_button = QPushButton('Delete')
        delete_button.clicked.connect(self.delete_action_from_list)
        action_button_layout.addWidget(delete_button)
        
        left_panel.addLayout(action_button_layout)
        
        # ============================================================================
        # CENTER PANEL - Visualization (Larger)
        # ============================================================================
        center_panel = QVBoxLayout()
        center_panel.setSpacing(0)
        center_panel.setContentsMargins(0, 0, 0, 0)
        
        center_header = QLabel("Visualization")
        center_header.setStyleSheet("font-weight: bold; color: #333; padding: 2px;")
        center_panel.addWidget(center_header)
        
        self.visualization_widget = VisualizationWidget()
        self.visualization_widget.setMinimumSize(400, 400)
        center_panel.addWidget(self.visualization_widget, 1)
        
        # ============================================================================
        # RIGHT PANEL - Parameters
        # ============================================================================
        right_panel = QVBoxLayout()
        right_panel.setSpacing(0)
        right_panel.setContentsMargins(0, 0, 0, 0)
        
        right_header = QLabel("Parameters")
        right_header.setStyleSheet("font-weight: bold; color: #333; padding: 2px;")
        right_panel.addWidget(right_header)
        
        self.action_parameter_layout = QVBoxLayout()
        self.action_parameter_layout.setSpacing(8)
        self.action_parameter_layout.setContentsMargins(5, 5, 5, 5)
        
        param_scroll = QScrollArea()
        param_scroll.setWidgetResizable(True)
        param_scroll.setStyleSheet("""
            QScrollArea {
                border: 1px solid #ddd;
                background-color: #fafafa;
            }
        """)
        param_container = QWidget()
        param_container.setLayout(self.action_parameter_layout)
        param_scroll.setWidget(param_container)
        right_panel.addWidget(param_scroll, 1)
        
        # ============================================================================
        # Assemble content layout
        # ============================================================================
        content_layout.addLayout(left_panel, 0)  # Left: fixed width (minimal)
        content_layout.addLayout(center_panel, 2)  # Center: larger
        content_layout.addLayout(right_panel, 1)  # Right: medium
        
        main_layout.addLayout(content_layout, 1)
        
        self.setLayout(main_layout)
        self.set_dispenser_ui_from_action_list()

    def save_file(self):
        """
        Save the current vision pipeline to a file.
        The file will be saved in the current directory with a timestamp.
        """
        if self.dispenser_action_list.file_path is None:
            # If no file path is set, create a new file
            self.create_new_file()
            
            #self.dispenser_action_list.file_path = "dispense_sequence.json"
        # Check if the directory exists, if not create it
        
        
        # if not os.path.exists(os.path.dirname(self.dispenser_action_list.file_path)):
        #     os.makedirs(os.path.dirname(self.dispenser_action_list.file_path))
            
        # Save the sequence to the file
        success = self.dispenser_action_list.save_to_file()
        
        if success:
            self.file_saved.emit(self.dispenser_action_list.file_path)
        # if success:
        #     self.text_output.append(f"Saved process to: {self.dispenser_action_list.file_path}")
        # else:
        #     self.text_output.append("Failed to save process. Please check the file path and permissions.")
    
    def save_file_as(self):
        """
        Save the current sequence to a new file using a save dialog.
        """
        # Set the file filters to show only PMDISPP files
        file_filter = "PMDISPP Files (*.pmdispp)"
        file_name, _ = QFileDialog.getSaveFileName(
            parent=self,
            caption="Save PMDISPP File As",
            filter=file_filter,
        )

        if file_name:
            if not file_name.endswith('.pmdispp'):
                file_name += '.pmdispp'
            
            self.dispenser_action_list.file_path = file_name
            success = self.dispenser_action_list.save_to_file()
            
            if success:
                self.set_widget_pipeline_name(os.path.basename(file_name))
                self.file_saved.emit(file_name)
        
    def show_vision_action_menu(self):
        self.contextMenu = QMenu()
        self.create_menu_from_data(self.contextMenu, self.dispenser_action_list.get_list_available_actions())
        self.contextMenu.exec(self.cursor().pos())

    def create_menu_from_dict(self, menu, menu_dict):
        for key, value in menu_dict.items():
            if isinstance(value, dict):
                submenu = menu.addMenu(key)
                self.create_menu_from_dict(submenu, value)
            elif isinstance(value, list):
                for item in value:
                    menu.addAction(item)

    def create_menu_from_data(self, menu, menu_data):
        """
        Create a context menu from a list of action names.
        Each action name will be added as an action to the menu.
        """
        for action_name in menu_data:
            action = QAction(action_name, self)
            action.triggered.connect(partial(self.add_function_to_pipeline, action_name))
            menu.addAction(action)

    def set_widget_pipeline_name(self, text):
        self.pipeline_name_widget.setText("File: " + text)

    def open_process_file(self, file_path_load=None):

        if not file_path_load:
            file_filter = "PMDISPP Files (*.pmdispp);;All Files (*)"
            file_path, _ = QFileDialog.getOpenFileName(
                parent=self,
                caption="Open PMDISPP File",
                #directory=self.current_vision_pipeline.vision_pipeline_json_dir,
                filter=file_filter,
            )
        else:
            file_path = file_path_load

        self.dispenser_action_list.file_path = os.path.dirname(
            file_path
        )
        
        if file_path:
            file_name = os.path.basename(file_path)
            success = self.dispenser_action_list.load_from_file(file_path)
            # initialize the ui with the new pipeline
            if success:
                self.set_dispenser_ui_from_action_list()
                self.set_widget_pipeline_name(file_name)
                self.set_visualization_image()
                self.file_loaded.emit(file_path)


    def set_visualization_image(self):
        
        fig = self.dispenser_action_list.get_visualization()

        if fig is not None:
            self.visualization_widget.set_figure(fig)

    def create_new_file(self):
        if self.dispenser_action_list.file_path is not None:
            return

        # Set the file filters to show only PMDISPP files
        file_filter = "PMDISPP Files (*.pmdispp)"
        file_name, _ = QFileDialog.getSaveFileName(
            parent=self,
            caption="Save PMDISPP File",
            #directory=self.current_vision_pipeline.vision_pipeline_json_dir,
            filter=file_filter,
        )

        if file_name:
            if not file_name.endswith('.pmdispp'):
                file_name += '.pmdispp'
            self.dispenser_action_list.file_path = file_name
            self.dispenser_action_list.clear_actions()  # Clear the current actions
            self.set_widget_pipeline_name(os.path.basename(file_name))
            #self.text_output.append(f"New process created: {file_name}")
            self.set_dispenser_ui_from_action_list()
            self.sequence_changed.emit()

    def parameter_change_callback(self):
        print("Parameter changed in DictEditor")
        self.set_visualization_image()
        self.action_modified.emit()
        self.sequence_changed.emit()

    def add_function_to_pipeline(self,function_name:str):
        """
        Add a function to the current vision pipeline.
        The function is selected from the combo box and added to the pipeline.
        """
        if function_name in self.dispenser_action_list.get_list_available_actions():
            action_class = self.dispenser_action_list.available_actions[function_name]
            action_instance = action_class()
            action_instance.set_name(function_name)
            self.dispenser_action_list.add_action(action_instance)

            # Update the UI
            self.set_dispenser_ui_from_action_list()
            self.create_function_parameters_layout()
            
            self.action_added.emit(function_name)
            self.sequence_changed.emit()
            #self.text_output.append(f"Added function: {function_name}")

    def action_in_list_clicked(self):
        """
        Callback function for when an action in the list is clicked.
        This will update the function parameters layout based on the selected action.
        """
        self.create_function_parameters_layout()
        self.set_visualization_image()



    def set_dispenser_ui_from_action_list(self):
        """
        Set the UI elements based on the current vision pipeline.
        This includes updating the checkbox list with the names of the functions in the pipeline.
        """
        self.checkbox_list.clear()
        for action in self.dispenser_action_list.actions:
            action:BaseAction
            item = ReorderableCheckBoxListItem(action.name)
            item.setCheckState(Qt.CheckState.Checked)
            self.checkbox_list.addItem(item)

        # Clear and update function parameters layout
        self.remove_all_action_parameter_widgets_from_layout()
        self.create_function_parameters_layout()
        self.add_function_paramter_widgets_to_layout()

    def delete_action_from_list(self):
        """
        Delete the currently selected action from the action list.
        """
        current_row = self.checkbox_list.currentRow()
        if current_row >= 0:
            self.dispenser_action_list.delete_action_at_index(current_row)
            self.checkbox_list.takeItem(current_row)
            self.remove_all_action_parameter_widgets_from_layout()
            self.create_function_parameters_layout()
            self.add_function_paramter_widgets_to_layout()
            
            self.action_removed.emit(current_row)
            self.sequence_changed.emit()
            #self.text_output.append(f"Deleted action at index: {current_row}")
    
    def create_function_parameters_layout(self):
        """
        Create the layout for the function parameters based on the currently selected action.
        This will create widgets for each parameter of the selected action.
        """
        self.remove_all_action_parameter_widgets_from_layout()

        current_row = self.checkbox_list.currentRow()
        if current_row >= 0:
            action = self.dispenser_action_list.actions[current_row]
            self.dispenser_action_list.enable_action_selection(current_row)
            action:BaseAction
            action_parameters = action.get_dict()

            # Create a DictEditor for the action parameters
            dict_editor = DictEditor(action_parameters, self.parameter_change_callback)
            self.action_parameter_layout.addWidget(dict_editor)
            self.action_parameter_widgets.append(dict_editor)
            # Connect the DictEditor to update the action parameters

    def add_function_paramter_widgets_to_layout(self):
        for widget in self.action_parameter_widgets:
            self.action_parameter_layout.addWidget(widget)

    def remove_all_action_parameter_widgets_from_layout(self):
        for widget in self.action_parameter_widgets:
            self.action_parameter_layout.removeWidget(widget)
            widget.deleteLater()
        self.action_parameter_widgets.clear()


    def on_drop(self):
        self.dispenser_action_list.move_action_to_index(
            old_index=self.checkbox_list.drag_source_position,
            new_index=self.checkbox_list.currentRow(),
        )
        self.create_function_parameters_layout()
        self.set_dispenser_ui_from_action_list()
        self.sequence_changed.emit()

    # ============================================================================
    # Public Integration API Methods - Use these to integrate with other apps
    # ============================================================================
    
    def get_sequence_generator(self) -> DispenseSequenceGenerator:
        """
        Get the underlying DispenseSequenceGenerator object.
        Use this for programmatic access to the sequence.
        
        Returns:
            DispenseSequenceGenerator: The current sequence generator
        """
        return self.dispenser_action_list
    
    def get_sequence_dict(self) -> dict:
        """
        Get the current sequence as a dictionary.
        
        Returns:
            dict: Dictionary representation of the sequence
        """
        return self.dispenser_action_list._get_sequence_dict()
    
    def get_current_file_path(self) -> str:
        """
        Get the current file path.
        
        Returns:
            str: Current file path or None if not set
        """
        return self.dispenser_action_list.file_path
    
    def set_file_path(self, file_path: str):
        """
        Set the file path without loading the file.
        
        Args:
            file_path: Path to set as the current file path
        """
        if file_path:
            self.dispenser_action_list.file_path = file_path
            self.set_widget_pipeline_name(os.path.basename(file_path))
    
    def load_sequence_from_file(self, file_path: str) -> bool:
        """
        Load a sequence from a file programmatically.
        
        Args:
            file_path: Path to the PMDISPP file to load
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if os.path.exists(file_path):
                success = self.dispenser_action_list.load_from_file(file_path)
                if success:
                    self.set_dispenser_ui_from_action_list()
                    self.set_widget_pipeline_name(os.path.basename(file_path))
                    self.set_visualization_image()
                    self.file_loaded.emit(file_path)
                return success
        except Exception as e:
            print(f"Error loading sequence: {e}")
        return False
    
    def save_sequence_to_file(self, file_path: str = None, generate_gcode: bool = True) -> bool:
        """
        Save the current sequence to a file programmatically.
        
        Args:
            file_path: Path to save to. If None, uses current file path
            generate_gcode: Whether to also generate and save G-code
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if file_path:
                self.dispenser_action_list.file_path = file_path
            
            if not self.dispenser_action_list.file_path:
                print("File path not set. Use set_file_path() first.")
                return False
            
            success = self.dispenser_action_list.save_to_file()
            
            
            if success:
                self.file_saved.emit(self.dispenser_action_list.file_path)
            
            return success
        except Exception as e:
            print(f"Error saving sequence: {e}")
        return False
    
    def export_gcode(self, start_point: Point = None) -> str:
        """
        Export G-code for the current sequence without saving to file.
        
        Args:
            start_point: Starting point for G-code generation (default: Point())
            
        Returns:
            str: Generated G-code as string
        """
        if start_point is None:
            start_point = Point()
        return self.dispenser_action_list.generate_g_code(start_point)
    
    def get_available_actions(self) -> list:
        """
        Get list of available action types that can be added.
        
        Returns:
            list: List of action type names
        """
        return self.dispenser_action_list.get_list_available_actions()
    
    def add_action_by_name(self, action_name: str) -> bool:
        """
        Add an action to the sequence by name.
        
        Args:
            action_name: Name of the action type to add
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if action_name in self.dispenser_action_list.get_list_available_actions():
                action_class = self.dispenser_action_list.available_actions[action_name]
                action_instance = action_class()
                action_instance.set_name(action_name)
                self.dispenser_action_list.add_action(action_instance)
                
                # Update the UI
                self.set_dispenser_ui_from_action_list()
                self.create_function_parameters_layout()
                
                self.action_added.emit(action_name)
                self.sequence_changed.emit()
                return True
        except Exception as e:
            print(f"Error adding action: {e}")
        return False
    
    def remove_action_by_index(self, index: int) -> bool:
        """
        Remove an action from the sequence by index.
        
        Args:
            index: Index of the action to remove
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if 0 <= index < len(self.dispenser_action_list.actions):
                self.dispenser_action_list.delete_action_at_index(index)
                self.checkbox_list.takeItem(index)
                self.remove_all_action_parameter_widgets_from_layout()
                self.create_function_parameters_layout()
                self.add_function_paramter_widgets_to_layout()
                
                self.action_removed.emit(index)
                self.sequence_changed.emit()
                return True
        except Exception as e:
            print(f"Error removing action: {e}")
        return False
    
    def clear_sequence(self):
        """Clear all actions from the sequence."""
        self.dispenser_action_list.clear_actions()
        self.set_dispenser_ui_from_action_list()
        self.sequence_changed.emit()
    
    def get_action_count(self) -> int:
        """Get the number of actions in the sequence."""
        return len(self.dispenser_action_list.actions)
    
    def get_action_at_index(self, index: int) -> BaseAction:
        """
        Get an action from the sequence by index.
        
        Args:
            index: Index of the action to retrieve
            
        Returns:
            BaseAction: The action object or None if index is out of range
        """
        try:
            if 0 <= index < len(self.dispenser_action_list.actions):
                return self.dispenser_action_list.actions[index]
        except Exception as e:
            print(f"Error getting action: {e}")
        return None


class MatplotlibCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def clear_plot(self):
        self.ax.clear()


class VisualizationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.canvas = None
        self._create_default_figure()

    def _create_default_figure(self):
        """Create a default empty figure with 50x50mm grid"""
        fig = Figure(figsize=(6, 6), dpi=100)
        ax = fig.add_subplot(111)
        
        # Set up a 50x50mm empty workspace
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 50)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Dispense Sequence Visualization (Empty)')
        
        # Add light background
        ax.set_facecolor('#f5f5f5')
        
        self.set_figure(fig)

    def set_figure(self, fig: plt.Figure):
        if self.canvas:
            self.layout.removeWidget(self.canvas)
            self.canvas.setParent(None)
            self.canvas.deleteLater()

        self.canvas = FigureCanvas(fig)
        self.layout.addWidget(self.canvas)
        self.canvas.draw()


class DictEditor(QWidget):
    def __init__(self, data: dict, change_callback_function, parent=None):
        super().__init__(parent)
        self.data = data
        self.editors = {}
        self.change_callback_function = change_callback_function
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        for key, value in self.data.items():
            if key == '_is_selected':
                continue
            
            # Create a parameter group/card
            param_frame = QWidget()
            param_frame.setStyleSheet("""
                QWidget {
                    background-color: white;
                    border: 1px solid #e0e0e0;
                    border-radius: 4px;
                    padding: 8px;
                    margin: 2px 0px;
                }
            """)
            param_layout = QHBoxLayout()
            param_layout.setContentsMargins(8, 4, 8, 4)
            param_layout.setSpacing(10)
            
            # Label - formatted nicely
            label = QLabel(str(key))
            label.setStyleSheet("""
                QLabel {
                    font-weight: 500;
                    color: #333;
                    min-width: 100px;
                }
            """)
            label_font = label.font()
            label_font.setPointSize(9)
            label.setFont(label_font)
            param_layout.addWidget(label)
            
            # Editor
            editor = self.create_editor(value)
            editor.setStyleSheet("""
                QLineEdit, QSpinBox, QDoubleSpinBox {
                    border: 1px solid #ccc;
                    border-radius: 3px;
                    padding: 4px;
                    background-color: #ffffff;
                }
                QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                    border: 2px solid #2196F3;
                    padding: 3px;
                }
                QCheckBox {
                    spacing: 5px;
                }
            """)
            self.editors[key] = editor
            param_layout.addWidget(editor, 1)
            
            param_frame.setLayout(param_layout)
            main_layout.addWidget(param_frame)
        
        main_layout.addStretch()
        self.setLayout(main_layout)

    def create_editor(self, value):
        if isinstance(value, bool):
            editor = QCheckBox()
            editor.setChecked(value)
            editor.stateChanged.connect(lambda _: self.update_data())
            editor.stateChanged.connect(self.change_callback_function)
        elif isinstance(value, int):
            editor = QSpinBox()
            editor.setRange(-2147483648, 2147483647)
            editor.setValue(value)
            editor.valueChanged.connect(lambda _: self.update_data())
            editor.valueChanged.connect(self.change_callback_function)
        elif isinstance(value, float):
            editor = QDoubleSpinBox()
            editor.setRange(-1e10, 1e10)
            editor.setDecimals(4)
            editor.setValue(value)
            editor.valueChanged.connect(lambda _: self.update_data())
            editor.valueChanged.connect(self.change_callback_function)
        else:
            editor = QLineEdit(str(value))
            editor.textChanged.connect(lambda _: self.update_data())
            editor.textChanged.connect(self.change_callback_function)
        return editor

    def update_data(self):
        for key, editor in self.editors.items():
            if isinstance(editor, QCheckBox):
                self.data[key] = editor.isChecked()
            elif isinstance(editor, QSpinBox):
                self.data[key] = editor.value()
            elif isinstance(editor, QDoubleSpinBox):
                self.data[key] = editor.value()
            elif isinstance(editor, QLineEdit):
                self.data[key] = editor.text()

    def get_data(self):
        self.update_data()
        return self.data
    
class ReorderableCheckBoxListWidget(QListWidget):
    CustDragSig = QtCore.pyqtSignal()

    def __init__(self):
        super(ReorderableCheckBoxListWidget, self).__init__()
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QListWidget.DragDropMode.InternalMove)

    def get_widget_list_names(self):
        name_list = []
        for index in range(self.count()):
            item = self.item(index)
            if item:
                name_list.append(item.text())
        return name_list

    def dropEvent(self, event):
        super(ReorderableCheckBoxListWidget, self).dropEvent(event)
        event.accept()
        self.CustDragSig.emit()

    def dragEnterEvent(self, event):
        self.drag_source_position = self.currentRow()  # Capture the source position
        super(ReorderableCheckBoxListWidget, self).dragEnterEvent(event)


class ReorderableCheckBoxListItem(QListWidgetItem):
    def __init__(self, function_name):
        super().__init__(function_name)
        self.setFlags(self.flags() | Qt.ItemFlag.ItemIsUserCheckable)



if __name__ == "__main__":
    # This demonstrates both standalone usage and integration patterns
    
    app = QApplication(sys.argv)

    # ========== STANDALONE USAGE ==========
    # Just run it as a window with menu bar
    window = DispenserBuilderMainApp()
    window.show()
    
    # ========== INTEGRATION USAGE EXAMPLES ==========
    # To integrate DispenserBuilderWidget into another application:
    #
    # 1. Import the widget:
    #    from pm_dispense_path_generator_app import DispenserBuilderWidget
    #
    # 2. Create and embed it in your app:
    #    dispense_widget = DispenserBuilderWidget()
    #    your_layout.addWidget(dispense_widget)
    #
    # 3. Connect to signals to react to changes:
    #    dispense_widget.file_loaded.connect(on_file_loaded)
    #    dispense_widget.sequence_changed.connect(on_sequence_changed)
    #    dispense_widget.action_added.connect(on_action_added)
    #
    # 4. Use public methods for programmatic control:
    #    dispense_widget.load_sequence_from_file("path/to/file.pmdispp")
    #    dispense_widget.add_action_by_name("MoveAction")
    #    sequence_dict = dispense_widget.get_sequence_dict()
    #    gcode = dispense_widget.export_gcode()
    #    dispense_widget.save_sequence_to_file("path/to/output.pmdispp")
    
    sys.exit(app.exec())