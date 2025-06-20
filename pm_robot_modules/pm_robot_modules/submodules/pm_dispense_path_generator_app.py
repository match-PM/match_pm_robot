
import sys
from PyQt6.QtCore import Qt
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

        # Create a menu bar
        menubar = self.menuBar()

        file_menu = menubar.addMenu("File")
        
        save_action = QAction("Save process", self)
        save_action.triggered.connect(self.main_widget.save_file)
        file_menu.addAction(save_action)
        
        open_action = QAction("Open process", self)
        open_action.triggered.connect(self.main_widget.open_process_file)
        file_menu.addAction(open_action)

        # Create "New" action
        new_action = QAction("New process", self)
        new_action.triggered.connect(self.main_widget.create_new_file)
        file_menu.addAction(new_action)

        # Create 'save as' action
        #save_as_action = QAction("Save process as", self)
        #save_as_action.triggered.connect(self.main_widget.save_process_as)

        #file_menu.addAction(save_as_action)

        central_widget.setLayout(main_layout)
        self.setGeometry(100, 100, 1000, 800)

class DispenserBuilderWidget(QWidget):
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

        # Create layout for the main container widget
        # layout = QVBoxLayout()
        layout = QGridLayout()

        self.pipeline_name_widget = QLabel()
        self.set_widget_pipeline_name("Not saved yet!")
        layout.addWidget(self.pipeline_name_widget, 0, 0, 1, 2)


        open_action_menu_button = QPushButton('Add \n Action')
        open_action_menu_button.clicked.connect(self.show_vision_action_menu)
        layout.addWidget(open_action_menu_button, 1, 0)

        # Add combobox for vision pipline building
        self.checkbox_list = ReorderableCheckBoxListWidget()
        self.checkbox_list.itemClicked.connect(self.action_in_list_clicked)
        #self.checkbox_list.itemChanged.connect(self.set_function_states)
        self.checkbox_list.CustDragSig.connect(self.on_drop)
        self.checkbox_list.itemChanged.connect(self.parameter_change_callback)
        layout.addWidget(self.checkbox_list, 2, 0)

        # add button for deleting selected vision function
        delete_button = QPushButton("Delete Selected")
        delete_button.clicked.connect(self.delete_action_from_list)
        layout.addWidget(delete_button, 3, 0)

        # add textbox for string output
        #self.text_output = AppTextOutput()
        #layout.addWidget(self.text_output, 4, 0, 1, 2)

        # create sub layout for the function parameter widgetd
        self.action_parameter_layout = QVBoxLayout()
        # add a textlabel to the sublayout
        self.action_parameter_layout.addWidget(QLabel("Function parameters:"))

        # add sublayout to app layout
        layout.addLayout(self.action_parameter_layout, 2, 1, Qt.AlignmentFlag.AlignTop)

        self.visualization_widget = VisualizationWidget()
        layout.addWidget(self.visualization_widget, 1, 2, 2, 2)

        self.setLayout(layout)
        self.setGeometry(100, 100, 600, 800)
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
        self.dispenser_action_list.save_g_code_to_file(start_joint_values=Point())
        # if success:
        #     self.text_output.append(f"Saved process to: {self.dispenser_action_list.file_path}")
        # else:
        #     self.text_output.append("Failed to save process. Please check the file path and permissions.")
        
        
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
        self.pipeline_name_widget.setText("Process name: " + text)

    def open_process_file(self, file_path_load=None):

        if not file_path_load:
            file_filter = "JSON Files (*.json);;All Files (*)"
            file_path, _ = QFileDialog.getOpenFileName(
                parent=self,
                caption="Open JSON File",
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

    # def save_process_as(self):
    #     self.save_as = True
    #     self.create_new_file()

    def set_visualization_image(self):
        
        fig = self.dispenser_action_list.get_visualization()

        if fig is not None:
            self.visualization_widget.set_figure(fig)

    def create_new_file(self):
        if self.dispenser_action_list.file_path is not None:
            return

        # Set the file filters to show only JSON files
        file_filter = "JSON Files (*.json)"
        file_name, _ = QFileDialog.getSaveFileName(
            parent=self,
            caption="Save JSON File",
            #directory=self.current_vision_pipeline.vision_pipeline_json_dir,
            filter=file_filter,
        )

        if file_name:
            if not file_name.endswith('.json'):
                file_name += '.json'
            self.dispenser_action_list.file_path = file_name
            self.dispenser_action_list.clear_actions()  # Clear the current actions
            self.set_widget_pipeline_name(os.path.basename(file_name))
            #self.text_output.append(f"New process created: {file_name}")
            self.set_dispenser_ui_from_action_list()

    def parameter_change_callback(self):
        print("Parameter changed in DictEditor")
        self.set_visualization_image()

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
        self.canvas = None  # placeholder for the figure canvas

    def set_figure(self, fig: plt.Figure):
        if self.canvas:
            self.layout.removeWidget(self.canvas)
            self.canvas.setParent(None)

        self.canvas = FigureCanvas(fig)
        self.layout.addWidget(self.canvas)
        self.canvas.draw()


class DictEditor(QWidget):
    def __init__(self, data: dict, change_callback_function, parent=None):
        super().__init__(parent)
        self.data = data
        self.editors = {}
        self.change_callback_function = change_callback_function  # Callback function to notify changes
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        for key, value in self.data.items():
            if key == '_is_selected':
                continue
            row_layout = QHBoxLayout()
            label = QLabel(str(key))
            row_layout.addWidget(label)

            editor = self.create_editor(value)
            self.editors[key] = editor
            row_layout.addWidget(editor)

            layout.addLayout(row_layout)

        self.setLayout(layout)

    def create_editor(self, value):
        if isinstance(value, bool):
            editor = QCheckBox()
            editor.setChecked(value)
            editor.stateChanged.connect(lambda _: self.update_data())
            editor.stateChanged.connect(self.change_callback_function)  # Notify change
        elif isinstance(value, int):
            editor = QSpinBox()
            editor.setValue(value)
            editor.valueChanged.connect(lambda _: self.update_data())
            editor.valueChanged.connect(self.change_callback_function)  # Notify change
            editor.setRange(-2147483648, 2147483647)
        elif isinstance(value, float):
            editor = QDoubleSpinBox()
            editor.setValue(value)
            editor.valueChanged.connect(lambda _: self.update_data())
            editor.valueChanged.connect(self.change_callback_function)  # Notify change
            editor.setRange(-1e10, 1e10)
        else:
            editor = QLineEdit(str(value))
            editor.textChanged.connect(lambda _: self.update_data())
            editor.textChanged.connect(self.change_callback_function)  # Notify change
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
    # Example usage
    dispense_action = DispenseAction()
    dispense_action.goal_x = 10.0
    dispense_action.goal_y = 5.0
    dispense_action.goal_z = 2.0
    dispense_action.heigth = 1.0
    dispense_action.speed = 0.5

    move_action = MoveAction()
    move_action.goal_x = 15.0
    move_action.goal_y = 10.0
    move_action.goal_z = 3.0
    move_action.heigth = 1.5
    move_action.speed = 0.7

    dip_action = DipAction()
    dip_action.dip_depth = 0.2
    dip_action.dip_speed = 0.1




    sequence_generator = DispenseSequenceGenerator()
    #sequence_generator.add_action(dispense_action)
    #sequence_generator.add_action(move_action)
    #sequence_generator.add_action(dip_action)
    #sequence_generator.load_from_file("dispense_sequence.json")
    #sequence_generator.save_to_file("dispense_sequence.json")


    print(sequence_generator._get_sequence_dict())



    app = QApplication(sys.argv)

    window = DispenserBuilderMainApp()
    window.show()
    sys.exit(app.exec())
    # for action in sequence_generator.actions:
    #     print(action.get_dict())