# Quick Refactoring Guide for Your ROS2 Components

This guide shows how to update your existing pm_client usage step-by-step.

## 1. Hardware Interface (`pm_hardware_interface/src/pm_system.cpp`)

### Current Code (Problem Areas)
```cpp
// on_configure()
try {
    m_pm_client.connect(m_config.opcua_endpoint);
}
catch (std::exception &e) {
    RCLCPP_ERROR(..., "Connection to OPCUA server failed: %s.", e.what());
    return CallbackReturn::ERROR;
}

// on_activate() - NO ERROR HANDLING
auto &robot = m_pm_client.get_robot();
for (auto &axis : m_axes) {
    axis.read(robot);  // What if this fails?
}
m_camera1_coax_light_state = robot.camera1->get_coax_light();  // Crashes if null
```

### Improved Code (With Error Handling)
```cpp
// on_configure() - ENHANCED
try {
    PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
    RCLCPP_INFO(logger, "Connecting to OPC UA server: %s", m_config.opcua_endpoint.c_str());
    m_pm_client.connect(m_config.opcua_endpoint);
    RCLCPP_INFO(logger, "Connected successfully");
}
catch (const PMClient::OpcuaException &e) {
    if (PMClient::is_connection_error(e.status_code())) {
        RCLCPP_FATAL(logger, "Cannot connect - is OPC UA server running? Error: %s", e.what());
    } else {
        RCLCPP_ERROR(logger, "Connection failed: %s", e.what());
    }
    return CallbackReturn::ERROR;
}
catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Unexpected error: %s", e.what());
    return CallbackReturn::ERROR;
}

// on_activate() - ENHANCED WITH VALIDATION
try {
    auto &robot = m_pm_client.get_robot();
    
    // Validate components are initialized
    if (!robot.camera1) {
        RCLCPP_ERROR(logger, "Camera1 is null - initialization failed");
        return CallbackReturn::ERROR;
    }
    if (!robot.camera1->is_ok()) {
        RCLCPP_ERROR(logger, "Camera1 nodes not properly initialized");
        return CallbackReturn::ERROR;
    }
    
    // Read with error handling
    for (auto &axis : m_axes) {
        try {
            axis.read(robot);
        }
        catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(logger, "Failed to read axis: %s", e.what());
            return CallbackReturn::ERROR;
        }
    }
    
    // Safe camera read
    try {
        m_camera1_coax_light_state = static_cast<double>(robot.camera1->get_coax_light());
    }
    catch (const PMClient::OpcuaException &e) {
        RCLCPP_WARN(logger, "Failed to read coax light: %s", e.what());
        m_camera1_coax_light_state = 0.0;
    }
    
    RCLCPP_INFO(logger, "Successfully activated PMSystem");
    return CallbackReturn::SUCCESS;
}
catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
}

// read() - ENHANCED WITH ERROR HANDLING
hardware_interface::return_type PMSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    auto logger = rclcpp::get_logger("PMSystem");

    try {
        auto &robot = m_pm_client.get_robot();
        
        // Check robot is valid
        if (!robot.x_axis) {
            RCLCPP_ERROR_THROTTLE(logger, *get_clock(), 1000, "X-axis not initialized");
            return hardware_interface::return_type::ERROR;
        }

        // Read axes with error catching (don't fail entire read if one fails)
        for (auto &axis : m_axes) {
            try {
                axis.read(robot);
            }
            catch (const PMClient::OpcuaException &e) {
                RCLCPP_WARN_THROTTLE(logger, *get_clock(), 5000, 
                                     "Failed to read axis %d: %s", 
                                     static_cast<int>(axis.get_axis_id()), e.what());
                // Continue reading other components
            }
        }

        // Read camera with safety checks
        if (robot.camera1 && robot.camera1->is_ok()) {
            try {
                m_camera1_coax_light_state = static_cast<double>(robot.camera1->get_coax_light());
            }
            catch (const PMClient::OpcuaException &e) {
                RCLCPP_DEBUG(logger, "Camera read failed, retrying next cycle: %s", e.what());
            }
        }

        return hardware_interface::return_type::OK;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Unexpected error in read(): %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

// write() - ENHANCED WITH VALIDATION
hardware_interface::return_type PMSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    auto logger = rclcpp::get_logger("PMSystem");

    try {
        auto &robot = m_pm_client.get_robot();
        
        if (!robot.x_axis || !robot.x_axis->is_ok()) {
            RCLCPP_ERROR_THROTTLE(logger, *get_clock(), 1000, "X-axis not available");
            return hardware_interface::return_type::ERROR;
        }

        // Check for errors before commanding
        if (robot.x_axis->get_error()) {
            RCLCPP_WARN(logger, "X-axis error %d - cannot command", robot.x_axis->get_error_id());
            return hardware_interface::return_type::ERROR;
        }

        // Write commands with error handling
        try {
            for (auto &axis : m_axes) {
                axis.write(robot);
            }
        }
        catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(logger, "Failed to write axis command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Unexpected error in write(): %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}
```

## 2. Lights Controller (Already Has Interface Abstraction)

The lights controller uses hardware interfaces, so error handling happens in pm_system.
**No changes needed** - but you CAN add validation if directly using pm_client:

```cpp
// If directly accessing robot (not recommended):
try {
    auto &robot = m_pm_client.get_robot();
    if (robot.camera1 && robot.camera1->is_ok()) {
        robot.camera1->set_coax_light(true);
    }
} catch (const PMClient::OpcuaException &e) {
    RCLCPP_ERROR(get_logger(), "Failed to set light: %s", e.what());
}
```

## 3. Moving to Error Handling Step by Step

### Step 1: Add Required Includes
```cpp
#include "pm_client/error_handling.hpp"  // NEW
#include "pm_client/debug.hpp"            // NEW
```

### Step 2: Configure Logging at Startup
```cpp
// In on_configure() or main():
PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
RCLCPP_DEBUG(logger, "PM Client debug logging enabled");
```

### Step 3: Wrap Calls in Try-Catch
Find all calls like:
```cpp
robot.x_axis->set_speed(100);
```

Wrap them:
```cpp
try {
    robot.x_axis->set_speed(100);
} catch (const PMClient::OpcuaException &e) {
    RCLCPP_ERROR(logger, "Failed to set speed: %s", e.what());
    // Handle error
}
```

### Step 4: Add Null Checks Before Use
```cpp
// Before: assumes always valid
auto &robot = m_pm_client.get_robot();
robot.x_axis->move(100);

// After: validates first
auto &robot = m_pm_client.get_robot();
if (!robot.x_axis || !robot.x_axis->is_ok()) {
    RCLCPP_ERROR(logger, "X-axis not initialized");
    return;
}
robot.x_axis->move(100);
```

### Step 5: Use THROTTLE for Frequent Errors
```cpp
// Don't spam logs - throttle to once per 5 seconds:
RCLCPP_ERROR_THROTTLE(logger, *get_clock(), 5000, "Failed to read: %s", e.what());
```

## 4. Creating a Wrapper for Cleaner Code

Instead of repeating error handling everywhere, create a wrapper:

```cpp
class SafePMClient
{
private:
    PMClient::Client &m_client;
    rclcpp::Logger m_logger;

public:
    SafePMClient(PMClient::Client &client, const rclcpp::Logger &logger)
        : m_client(client), m_logger(logger) {}

    bool move_axis(PMClient::AxisId axis_id, int target)
    {
        try {
            auto &robot = m_client.get_robot();
            PMClient::AerotechAxis *axis = get_axis(robot, axis_id);
            
            if (!axis || !axis->is_ok()) {
                RCLCPP_ERROR(m_logger, "Axis not initialized");
                return false;
            }
            
            if (axis->get_error()) {
                RCLCPP_WARN(m_logger, "Axis has error %d", axis->get_error_id());
                return false;
            }
            
            axis->move(target);
            return true;
        }
        catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(m_logger, "OPC UA error: %s", e.what());
            return false;
        }
    }
    
    // Similar methods for other operations...
};

// Usage:
SafePMClient safe_client(m_pm_client, get_logger());
if (!safe_client.move_axis(AxisId::X, 1000)) {
    // Handle error
}
```

## 5. Testing the Changes

After updating, test with:

```bash
# Enable debug logging via parameter
ros2 run your_package your_node --ros-args -p pm_client_log_level:=debug

# Watch for:
# - Connection messages
# - Read/write operations
# - Error messages with context
# - Status codes
```

## 6. Monitoring and Debugging in Production

Create a debug service to check status:

```cpp
auto debug_service = this->create_service<pm_msgs::srv::DebugStatus>(
    "debug_status",
    [this](const auto &request, auto &response) {
        try {
            auto &robot = m_pm_client.get_robot();
            response->ok = true;
            
            if (robot.x_axis && robot.x_axis->is_ok()) {
                response->status += "X-axis: OK\n";
            } else {
                response->status += "X-axis: FAILED\n";
            }
            
            // Check all components...
        }
        catch (const std::exception &e) {
            response->ok = false;
            response->status = std::string("Error: ") + e.what();
        }
    }
);
```

Call it:
```bash
ros2 service call /debug_status pm_msgs/srv/DebugStatus {}
```

## Summary Checklist

- [ ] Add `#include "pm_client/error_handling.hpp"` and `#include "pm_client/debug.hpp"`
- [ ] Configure DebugLogger at startup
- [ ] Add try-catch around pm_client calls
- [ ] Add null/validity checks before using robot components
- [ ] Replace ERROR returns with proper error messages
- [ ] Test with debug logging enabled
- [ ] Monitor logs for timeout errors
- [ ] Add status checking before operations
- [ ] Create debug callback for production support
