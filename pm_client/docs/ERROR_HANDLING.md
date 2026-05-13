# PM Client Error Handling & Debugging Improvements

## Overview

This document describes the improvements made to the pm_client library to provide better error handling and debugging capabilities for ROS2-based applications.

## Problems Addressed

### Before
1. **Silent Errors**: Failed read/write operations returned default values (e.g., 0 or false) without throwing exceptions
2. **No Logging**: No visibility into what operations were happening or why they failed
3. **Unclear Error Status**: Commented-out exception throws left errors unhandled
4. **Poor Debugging**: Difficult to diagnose issues in production or test environments
5. **Missing Context**: Errors provided no information about which operation failed or where

### After
1. **Explicit Exceptions**: All failures throw well-typed exceptions with detailed context
2. **Debug Logging**: Complete operation logging with configurable levels
3. **Status Code Analysis**: Helper functions to categorize and understand OPC UA status codes
4. **ROS2 Integration**: Easy integration with RCLCPP logging
5. **Rich Error Information**: Exceptions include status codes, operation names, and context

## New Components

### 1. Error Handling Module (`error_handling.hpp`)

Provides specialized exception types for different failure scenarios:

#### `OpcuaException`
- Thrown when OPC UA operations fail
- Includes status code and operation context
- Provides access to the underlying UA_StatusCode for analysis

```cpp
try {
    int speed = axis->get_speed();
} catch (const PMClient::OpcuaException &e) {
    // e.status_code() - Get the OPC UA status code
    // e.context() - Get additional context info
    // e.what() - Get formatted error message
    std::cerr << e.what() << std::endl;
}
```

#### `NodeTypeException`
- Thrown when a node has unexpected type or structure
- Indicates what type was expected vs. received

#### `InvalidNodeException`
- Thrown when a node ID is null or invalid
- Includes node name and context

#### `InitializationException`
- Thrown during client initialization
- Provides specific information about what failed during init

#### Helper Functions
```cpp
// Check status code results
PMClient::is_status_good(status);           // Is operation successful?
PMClient::is_connection_error(status);      // Connection problem?
PMClient::is_timeout_error(status);         // Timeout?
PMClient::status_code_to_string(status);    // Human-readable string
```

### 2. Debug Logging Module (`debug.hpp`)

Provides flexible logging infrastructure:

#### Log Levels
- `Debug` - Detailed operation logs
- `Info` - General informational messages
- `Warning` - Warning conditions
- `Error` - Error conditions
- `Critical` - Critical failures

#### Usage
```cpp
// Set global log level
PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);

// Log messages
PMClient::DebugLogger::debug("Category", "Debug message");
PMClient::DebugLogger::info("Category", "Info message");
PMClient::DebugLogger::warning("Category", "Warning message");
PMClient::DebugLogger::error("Category", "Error message");

// Log OPC UA status codes
PMClient::DebugLogger::log_status("Category", status, "operation_name", "context");

// Enable/disable console output
PMClient::DebugLogger::set_console_output(true);
```

### 3. Enhanced Client Methods

All read/write operations now:
- Check OPC UA status codes
- Validate node types
- Throw appropriate exceptions on failure
- Log detailed information at debug level

```cpp
// Before (silent failure):
int speed = client.read_node_value<int>(speed_node_id);  // Returns 0 on error

// After (explicit error):
try {
    int speed = client.read_node_value<int>(speed_node_id);
} catch (const PMClient::OpcuaException &e) {
    // Handle error with full context
}
```

## Integration with ROS2

### Basic Setup

```cpp
#include "rclcpp/rclcpp.hpp"
#include "pm_client/client.hpp"
#include "pm_client/debug.hpp"
#include "pm_client/error_handling.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : rclcpp::Node("my_node") {
        // Configure logging level from ROS parameters
        std::string log_level = this->declare_parameter<std::string>("pm_client_log_level", "warning");
        
        if (log_level == "debug") {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
        } else if (log_level == "info") {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Info);
        }
        
        try {
            m_client = std::make_unique<PMClient::Client>();
            m_client->connect("opc.tcp://localhost:4840");
            m_client->init();
            RCLCPP_INFO(get_logger(), "PM Client initialized");
        } catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize PM Client: %s", e.what());
            throw;
        }
    }
    
private:
    std::unique_ptr<PMClient::Client> m_client;
};
```

### Service Handler with Error Handling

```cpp
void handle_move_request(
    std::shared_ptr<pm_msgs::srv::MoveAxis::Request> request,
    std::shared_ptr<pm_msgs::srv::MoveAxis::Response> response)
{
    try {
        auto &robot = m_client->get_robot();
        if (!robot.x_axis || !robot.x_axis->is_ok()) {
            response->success = false;
            response->message = "X-axis not initialized";
            RCLCPP_ERROR(get_logger(), "X-axis not properly initialized");
            return;
        }
        
        if (robot.x_axis->get_error()) {
            response->success = false;
            response->message = "Axis error: " + std::to_string(robot.x_axis->get_error_id());
            RCLCPP_WARN(get_logger(), "X-axis error ID: %d", robot.x_axis->get_error_id());
            return;
        }
        
        robot.x_axis->move(request->target);
        response->success = true;
        response->message = "Move sent";
    } catch (const PMClient::OpcuaException &e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(get_logger(), "Move failed: %s", e.what());
    }
}
```

## Migration Guide

### For Existing Code

If you have existing code using pm_client, update it as follows:

**Before:**
```cpp
int speed = axis->get_speed();  // Silently returns 0 on error
if (speed == 0) {
    // Can't tell if this is valid or an error
}
```

**After:**
```cpp
try {
    int speed = axis->get_speed();
    // Use speed value
} catch (const PMClient::OpcuaException &e) {
    RCLCPP_ERROR(get_logger(), "Failed to read speed: %s", e.what());
    // Handle error appropriately
}
```

### Validation Pattern

Always validate components before use:

```cpp
auto &robot = client.get_robot();

// Check pointer is not null
if (!robot.x_axis) {
    RCLCPP_ERROR(get_logger(), "X-axis not available");
    return;
}

// Check node IDs are properly initialized
if (!robot.x_axis->is_ok()) {
    RCLCPP_ERROR(get_logger(), "X-axis not properly initialized");
    return;
}

// Now safe to use
robot.x_axis->move(target);
```

## Debugging Commands

### Enable Full Debug Logging

```cpp
PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
PMClient::DebugLogger::set_console_output(true);
```

Output will appear with timestamps and log levels:
```
[14:23:45] [INFO] [Client] Connecting to OPCUA server at: opc.tcp://localhost:4840
[14:23:45] [DEBUG] [Client] Successfully read scalar node value
[14:23:46] [WARN] [Client] Node read timeout
```

### Analyze Status Codes

```cpp
catch (const PMClient::OpcuaException &e) {
    if (PMClient::is_connection_error(e.status_code())) {
        // Handle connection loss
    } else if (PMClient::is_timeout_error(e.status_code())) {
        // Handle timeout
    } else {
        // Other error
    }
}
```

### Check Component Status

```cpp
// Comprehensive status check
if (robot.x_axis && robot.x_axis->is_ok()) {
    RCLCPP_INFO(get_logger(), "X-axis ready");
} else if (!robot.x_axis) {
    RCLCPP_ERROR(get_logger(), "X-axis pointer is null");
} else {
    RCLCPP_ERROR(get_logger(), "X-axis nodes not initialized");
}
```

## Testing & Validation

### Unit Test Pattern

```cpp
TEST(PMClientTest, ReadNodeThrowsOnFailure) {
    PMClient::DebugLogger::set_level(PMClient::LogLevel::Error);
    PMClient::Client client;
    
    // Connect to server...
    client.connect("opc.tcp://localhost:4840");
    
    // This should throw OpcuaException with bad node ID
    EXPECT_THROW({
        client.read_node_value<int>(UA_NODEID_NULL);
    }, PMClient::OpcuaException);
}
```

## Best Practices

1. **Always Use Try-Catch**: All pm_client operations may throw exceptions
2. **Check Initialization**: Use `is_ok()` methods before operating on components
3. **Log Operation Results**: Use DebugLogger for diagnostic information
4. **Handle Connection Errors**: Distinguish between temporary and permanent failures
5. **Validate Input**: Check parameters before passing to pm_client methods
6. **Use RCLCPP Logging**: Integrate with ROS2 logging for consistency
7. **Set Log Levels**: Configure logging level based on environment/parameter

## Example Applications

See [DEBUGGING_GUIDE.md](DEBUGGING_GUIDE.md) for complete example code including:
- ROS2 node setup with error handling
- Service handlers with proper validation
- Status checking patterns
- Debug callbacks for runtime inspection

## Additional Resources

- **OPC UA Status Codes**: Documented in open62541 library documentation
- **ROS2 Logging**: https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html
- **Exception Safety**: C++ exception handling best practices

## Future Improvements

Potential enhancements:
1. Integration with ROS2 diagnostics aggregator
2. Automatic reconnection with exponential backoff
3. Performance monitoring and metrics
4. Structured logging with JSON output
5. Remote debugging interface
