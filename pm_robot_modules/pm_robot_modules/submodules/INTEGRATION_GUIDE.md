# Dispense Path Generator Widget - Integration Guide

The `DispenserBuilderWidget` is now fully embeddable and can be integrated into other PyQt6 applications.

## Basic Embedding

### 1. Import the widget

```python
from pm_robot_modules.submodules.pm_dispense_path_generator_app import DispenserBuilderWidget
```

### 2. Create and add to your layout

```python
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout

class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        
        # Create and embed the dispense widget
        self.dispense_widget = DispenserBuilderWidget()
        layout.addWidget(self.dispense_widget)
        
        central_widget.setLayout(layout)
```

## Signals (Event Handling)

Connect to signals to react when things happen in the widget:

```python
# File operations
self.dispense_widget.file_loaded.connect(self.on_file_loaded)
self.dispense_widget.file_saved.connect(self.on_file_saved)

# Action management
self.dispense_widget.action_added.connect(self.on_action_added)
self.dispense_widget.action_removed.connect(self.on_action_removed)
self.dispense_widget.action_modified.connect(self.on_action_modified)

# Overall changes
self.dispense_widget.sequence_changed.connect(self.on_sequence_changed)

# Define your callbacks
def on_file_loaded(self, file_path):
    print(f"Sequence loaded from: {file_path}")

def on_sequence_changed(self):
    print("Sequence was modified")
```

## Public Methods

### File Operations

```python
# Load a sequence from file
success = self.dispense_widget.load_sequence_from_file("/path/to/file.pmdispp")

# Save sequence to file
success = self.dispense_widget.save_sequence_to_file("/path/to/output.pmdispp", generate_gcode=True)

# Set file path without loading
self.dispense_widget.set_file_path("/path/to/file.pmdispp")

# Get current file path
current_path = self.dispense_widget.get_current_file_path()
```

### Sequence Data Access

```python
# Get the underlying generator object
generator = self.dispense_widget.get_sequence_generator()

# Get sequence as dictionary
sequence_dict = self.dispense_widget.get_sequence_dict()

# Get number of actions
count = self.dispense_widget.get_action_count()

# Get specific action
action = self.dispense_widget.get_action_at_index(0)

# Export G-code without saving to file
gcode = self.dispense_widget.export_gcode()
```

### Action Management

```python
# Get available action types
available_actions = self.dispense_widget.get_available_actions()

# Add an action by name
success = self.dispense_widget.add_action_by_name("MoveAction")

# Remove an action by index
success = self.dispense_widget.remove_action_by_index(0)

# Clear all actions
self.dispense_widget.clear_sequence()
```

## Complete Example

```python
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import Qt
from pm_robot_modules.submodules.pm_dispense_path_generator_app import DispenserBuilderWidget

class DispenserApplication(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("My Dispenser Application")
        self.setGeometry(100, 100, 1200, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        
        # Create dispense widget
        self.dispense_widget = DispenserBuilderWidget()
        layout.addWidget(self.dispense_widget)
        
        # Connect signals
        self.dispense_widget.sequence_changed.connect(self.on_sequence_changed)
        self.dispense_widget.file_saved.connect(self.on_file_saved)
        
        # Add custom buttons
        button_layout = QVBoxLayout()
        
        export_btn = QPushButton("Export G-code")
        export_btn.clicked.connect(self.export_gcode)
        button_layout.addWidget(export_btn)
        
        layout.addLayout(button_layout)
        central_widget.setLayout(layout)
        
        self.status_label = QLabel("Ready")
        self.setStatusTip(self.status_label.text())
    
    def on_sequence_changed(self):
        count = self.dispense_widget.get_action_count()
        self.statusBar().showMessage(f"Sequence has {count} actions")
    
    def on_file_saved(self, file_path):
        self.statusBar().showMessage(f"Saved to: {file_path}")
    
    def export_gcode(self):
        gcode = self.dispense_widget.export_gcode()
        print("Generated G-code:")
        print(gcode)
        self.statusBar().showMessage("G-code exported to console")

if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    window = DispenserApplication()
    window.show()
    sys.exit(app.exec())
```

## Key Design Points

- **Separation of Concerns**: The `DispenserBuilderWidget` handles all UI logic, while the business logic is in `DispenseSequenceGenerator`
- **Signals**: Use signals to connect the widget to your application's logic without tight coupling
- **Public API**: All integration points are clearly marked with public methods
- **File Format**: Uses `.pmdispp` extension for dispense sequences and `.pmdispp.g` for generated G-code
- **Error Handling**: All public methods include try-catch blocks and return success/failure status

## Standalone vs Embedded

### Standalone Usage
```python
from pm_robot_modules.submodules.pm_dispense_path_generator_app import DispenserBuilderMainApp
app = DispenserBuilderMainApp()
app.show()
```

### Embedded Usage
```python
from pm_robot_modules.submodules.pm_dispense_path_generator_app import DispenserBuilderWidget
widget = DispenserBuilderWidget()
your_layout.addWidget(widget)
```

The `DispenserBuilderMainApp` is a QMainWindow wrapper that adds a menu bar for file operations. The `DispenserBuilderWidget` is the core reusable component.
