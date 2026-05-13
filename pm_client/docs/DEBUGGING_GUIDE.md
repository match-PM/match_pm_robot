/**
 * Best Practices & Debugging Guide for PM Client
 * ================================================
 * 
 * This document provides guidelines for using the pm_client with proper
 * error handling and debugging in ROS2.
 */

// ==============================================================================
// 1. ENABLING DEBUG LOGGING
// ==============================================================================

#include "pm_client/debug.hpp"
#include "pm_client/client.hpp"

// At the start of your application/node:
int main()
{
    // Enable debug logging and set minimum level
    PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
    PMClient::DebugLogger::set_console_output(true);
    
    // Now all pm_client operations will log detailed information
    PMClient::Client client;
    // ... rest of code
}

// ==============================================================================
// 2. PROPER ERROR HANDLING WITH TRY-CATCH
// ==============================================================================

// The new pm_client throws exceptions for all errors. Proper ROS2 integration:

#include "rclcpp/rclcpp.hpp"
#include "pm_client/error_handling.hpp"

class RobotController : public rclcpp::Node
{
public:
    RobotController() : rclcpp::Node("robot_controller")
    {
        try {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
            
            m_client = std::make_unique<PMClient::Client>();
            m_client->connect("opc.tcp://your_server:4840");
            m_client->init();
            
            RCLCPP_INFO(get_logger(), "PM Client initialized successfully");
        }
        catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(get_logger(), "OPC UA error: %s (code: 0x%x)", 
                         e.what(), e.status_code());
            throw;
        }
        catch (const PMClient::InitializationException &e) {
            RCLCPP_ERROR(get_logger(), "Initialization error: %s", e.what());
            throw;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Unexpected error: %s", e.what());
            throw;
        }
    }

    void move_axis(int target)
    {
        try {
            auto &robot = m_client->get_robot();
            
            // Validate axis is initialized
            if (!robot.x_axis || !robot.x_axis->is_ok()) {
                RCLCPP_ERROR(get_logger(), "X-axis not properly initialized");
                return;
            }
            
            // Check for axis errors before moving
            if (robot.x_axis->get_error()) {
                RCLCPP_WARN(get_logger(), "X-axis has error ID: %d", 
                           robot.x_axis->get_error_id());
                return;
            }
            
            robot.x_axis->move(target);
            RCLCPP_INFO(get_logger(), "X-axis move command sent to: %d", target);
        }
        catch (const PMClient::OpcuaException &e) {
            RCLCPP_ERROR(get_logger(), "Failed to move axis: %s", e.what());
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Unexpected error during axis move: %s", e.what());
        }
    }

private:
    std::unique_ptr<PMClient::Client> m_client;
};

// ==============================================================================
// 3. CHECKING COMPONENT STATUS BEFORE USE
// ==============================================================================

// All motor components have is_ok() methods to validate they were initialized:

auto &robot = client.get_robot();

// Check if an axis is properly initialized
if (!robot.x_axis) {
    RCLCPP_ERROR(get_logger(), "X-axis is nullptr");
    return;
}

if (!robot.x_axis->is_ok()) {
    RCLCPP_ERROR(get_logger(), "X-axis node IDs not properly initialized");
    return;
}

// Similar checks for other components
if (!robot.camera1 || !robot.camera1->is_ok()) {
    RCLCPP_ERROR(get_logger(), "Camera1 not properly initialized");
    return;
}

// ==============================================================================
// 4. EXCEPTION TYPES & HANDLING
// ==============================================================================

// OpcuaException - thrown for OPC UA communication failures
try {
    int speed = robot.x_axis->get_speed();
} catch (const PMClient::OpcuaException &e) {
    // Check if it's a connection error vs. other error
    if (PMClient::is_connection_error(e.status_code())) {
        RCLCPP_CRITICAL(get_logger(), "Lost connection to OPC UA server");
        // Attempt reconnection logic
    } else if (PMClient::is_timeout_error(e.status_code())) {
        RCLCPP_WARN(get_logger(), "OPC UA timeout - server may be slow");
    } else {
        RCLCPP_ERROR(get_logger(), "OPC UA error: %s", e.what());
    }
}

// NodeTypeException - thrown when node has wrong type/structure
try {
    auto values = client.read_node_values<int, 3>(some_node_id);
} catch (const PMClient::NodeTypeException &e) {
    RCLCPP_ERROR(get_logger(), "Node type mismatch: %s", e.what());
}

// InvalidNodeException - thrown when node ID is null/invalid
try {
    if (PMClient::UA_NodeId_isNull(&robot.x_axis->speed_node_id)) {
        throw PMClient::InvalidNodeException("speed_node_id for X-axis", "axis not initialized");
    }
} catch (const PMClient::InvalidNodeException &e) {
    RCLCPP_ERROR(get_logger(), "Invalid node: %s", e.what());
}

// InitializationException - thrown during client init if components missing
try {
    client.init();
} catch (const PMClient::InitializationException &e) {
    RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
}

// ==============================================================================
// 5. LOGGING & DEBUGGING OPERATIONS
// ==============================================================================

// Set log level based on ROS parameter
rclcpp::Parameter log_level_param("pm_client_log_level", "warning");
if (this->get_parameter("pm_client_log_level", log_level_param)) {
    std::string level_str = log_level_param.as_string();
    if (level_str == "debug") {
        PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
    } else if (level_str == "info") {
        PMClient::DebugLogger::set_level(PMClient::LogLevel::Info);
    } else if (level_str == "warning") {
        PMClient::DebugLogger::set_level(PMClient::LogLevel::Warning);
    } else if (level_str == "error") {
        PMClient::DebugLogger::set_level(PMClient::LogLevel::Error);
    }
}

// Use logging in your code
PMClient::DebugLogger::info("MyComponent", "Operation starting");
PMClient::DebugLogger::warning("MyComponent", "Timeout occurred, retrying...");
PMClient::DebugLogger::error("MyComponent", "Failed to communicate with device");

// ==============================================================================
// 6. RECOMMENDED ROS2 INTEGRATION PATTERNS
// ==============================================================================

// Example: Safe service handler with proper error handling
void pm_server_node::handle_move_request(
    std::shared_ptr<pm_msgs::srv::MoveAxis::Request> request,
    std::shared_ptr<pm_msgs::srv::MoveAxis::Response> response)
{
    try {
        auto &robot = m_client->get_robot();
        
        // Validate input
        if (request->target < -1000 || request->target > 1000) {
            response->success = false;
            response->message = "Target position out of range";
            RCLCPP_WARN(get_logger(), "Invalid target position: %d", request->target);
            return;
        }
        
        // Get the appropriate axis
        PMClient::AerotechAxis *axis = nullptr;
        switch (request->axis) {
            case 0: axis = robot.x_axis.get(); break;
            case 1: axis = robot.y_axis.get(); break;
            case 2: axis = robot.z_axis.get(); break;
            default:
                response->success = false;
                response->message = "Invalid axis";
                return;
        }
        
        if (!axis || !axis->is_ok()) {
            response->success = false;
            response->message = "Axis not initialized";
            RCLCPP_ERROR(get_logger(), "Axis %d not properly initialized", request->axis);
            return;
        }
        
        // Check for existing errors
        if (axis->get_error()) {
            response->success = false;
            response->message = "Axis has error: " + std::to_string(axis->get_error_id());
            RCLCPP_WARN(get_logger(), "Axis %d error ID: %d", request->axis, axis->get_error_id());
            return;
        }
        
        // Perform the move
        axis->move(request->target);
        response->success = true;
        response->message = "Move command sent";
        RCLCPP_INFO(get_logger(), "Axis %d move to %d successful", request->axis, request->target);
        
    } catch (const PMClient::OpcuaException &e) {
        response->success = false;
        response->message = std::string("OPC UA error: ") + e.what();
        RCLCPP_ERROR(get_logger(), "OPC UA error in move handler: %s", e.what());
    } catch (const std::exception &e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
        RCLCPP_ERROR(get_logger(), "Unexpected error in move handler: %s", e.what());
    }
}

// ==============================================================================
// 7. RUNTIME DEBUGGING SUPPORT
// ==============================================================================

// For runtime debugging in a ROS2 callback:
void debug_status_callback(std::shared_ptr<pm_msgs::srv::DebugStatus::Request> request,
                           std::shared_ptr<pm_msgs::srv::DebugStatus::Response> response)
{
    try {
        auto &robot = m_client->get_robot();
        std::stringstream info;
        
        // Check all axes
        if (robot.x_axis && robot.x_axis->is_ok()) {
            info << "X-Axis: OK (pos=" << robot.x_axis->get_actual_position() << ", err=" 
                 << robot.x_axis->get_error() << ")\n";
        } else {
            info << "X-Axis: NOT INITIALIZED\n";
        }
        
        // Check cameras
        if (robot.camera1 && robot.camera1->is_ok()) {
            info << "Camera1: OK\n";
        } else {
            info << "Camera1: NOT INITIALIZED\n";
        }
        
        response->status = info.str();
        response->ok = true;
    } catch (const std::exception &e) {
        response->status = std::string("Error: ") + e.what();
        response->ok = false;
    }
}
