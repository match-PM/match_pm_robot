/**
 * INTEGRATION GUIDE: Using PM Client with Error Handling in ROS2
 *
 * This guide shows how to integrate the new pm_client error handling
 * and debugging features into your ROS2 nodes and hardware interfaces.
 */

#include "pm_client/client.hpp"
#include "pm_client/debug.hpp"
#include "pm_client/error_handling.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pm_ros2_integration
{

// ==============================================================================
// PATTERN 1: HARDWARE INTERFACE WITH ERROR HANDLING
// ==============================================================================

/**
 * Enhanced version of PMSystem with proper error handling
 *
 * This shows how to wrap pm_client calls with try-catch and logging
 */
class EnhancedPMSystem : public hardware_interface::SystemInterface
{
  private:
    PMClient::Client m_pm_client;
    std::string m_logger_name;

  public:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        auto logger = rclcpp::get_logger("PMSystem");

        // Configure pm_client logging based on ROS parameter
        std::string log_level = "warning"; // Get from param if available
        if (log_level == "debug")
        {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
            RCLCPP_DEBUG(logger, "PM Client debug logging enabled");
        }
        else if (log_level == "info")
        {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Info);
        }

        // Connection with improved error handling
        try
        {
            RCLCPP_INFO(logger, "Connecting to OPC UA server...");
            m_pm_client.connect("opc.tcp://localhost:4840");
            RCLCPP_INFO(logger, "Successfully connected to OPC UA server");
        }
        catch (const PMClient::OpcuaException &e)
        {
            // Check connection-specific errors
            if (PMClient::is_connection_error(e.status_code()))
            {
                RCLCPP_FATAL(
                    logger,
                    "Cannot connect to OPC UA server - check if server is running and endpoint is "
                    "correct. Error: %s",
                    e.what()
                );
            }
            else if (PMClient::is_timeout_error(e.status_code()))
            {
                RCLCPP_ERROR(
                    logger,
                    "OPC UA connection timeout - server may be slow or unresponsive. Error: %s",
                    e.what()
                );
            }
            else
            {
                RCLCPP_ERROR(logger, "OPC UA connection error: %s", e.what());
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error during connection: %s", e.what());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // Initialization with component validation
        try
        {
            RCLCPP_INFO(logger, "Initializing PM Client...");
            m_pm_client.init();

            // Validate that all required components are initialized
            auto &robot = m_pm_client.get_robot();

            if (!robot.x_axis || !robot.x_axis->is_ok())
            {
                throw PMClient::InitializationException("X-axis not properly initialized");
            }
            if (!robot.y_axis || !robot.y_axis->is_ok())
            {
                throw PMClient::InitializationException("Y-axis not properly initialized");
            }
            if (!robot.z_axis || !robot.z_axis->is_ok())
            {
                throw PMClient::InitializationException("Z-axis not properly initialized");
            }
            if (!robot.camera1 || !robot.camera1->is_ok())
            {
                throw PMClient::InitializationException("Camera1 not properly initialized");
            }
            if (!robot.camera2 || !robot.camera2->is_ok())
            {
                throw PMClient::InitializationException("Camera2 not properly initialized");
            }

            RCLCPP_INFO(logger, "PM Client initialized successfully with all components validated");
        }
        catch (const PMClient::InitializationException &e)
        {
            RCLCPP_ERROR(logger, "PM Client initialization failed: %s", e.what());
            m_pm_client.disconnect();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch (const PMClient::OpcuaException &e)
        {
            RCLCPP_ERROR(logger, "OPC UA error during initialization: %s", e.what());
            m_pm_client.disconnect();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error during initialization: %s", e.what());
            m_pm_client.disconnect();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        auto logger = rclcpp::get_logger("PMSystem");

        try
        {
            auto &robot = m_pm_client.get_robot();

            // Add guards for null pointers
            if (!robot.x_axis)
            {
                RCLCPP_ERROR(logger, "X-axis is not initialized");
                return hardware_interface::return_type::ERROR;
            }

            // Read axis data with error handling
            try
            {
                int actual_pos = robot.x_axis->get_actual_position();
                // Use the position...
                RCLCPP_DEBUG(logger, "X-axis position: %d", actual_pos);
            }
            catch (const PMClient::OpcuaException &e)
            {
                if (PMClient::is_timeout_error(e.status_code()))
                {
                    RCLCPP_WARN(logger, "Timeout reading X-axis position - retrying on next cycle");
                }
                else
                {
                    RCLCPP_ERROR(logger, "Failed to read X-axis position: %s", e.what());
                }
                return hardware_interface::return_type::ERROR;
            }

            // Check axis for errors
            if (robot.x_axis->get_error())
            {
                RCLCPP_ERROR(logger, "X-axis has error: ID=%d", robot.x_axis->get_error_id());
                return hardware_interface::return_type::ERROR;
            }

            // Similarly for other components
            if (robot.camera1 && robot.camera1->is_ok())
            {
                try
                {
                    bool coax_light = robot.camera1->get_coax_light();
                    // Use coax_light value
                }
                catch (const PMClient::OpcuaException &e)
                {
                    RCLCPP_WARN(logger, "Failed to read camera1 coax light: %s", e.what());
                }
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error in read(): %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        auto logger = rclcpp::get_logger("PMSystem");

        try
        {
            auto &robot = m_pm_client.get_robot();

            if (!robot.x_axis || !robot.x_axis->is_ok())
            {
                RCLCPP_ERROR(logger, "X-axis not available for write");
                return hardware_interface::return_type::ERROR;
            }

            // Validate command within safe ranges
            int target = 1000; // Get from command interface
            if (target < robot.x_axis->get_min_position() ||
                target > robot.x_axis->get_max_position())
            {
                RCLCPP_ERROR(
                    logger,
                    "Target position %d out of range [%d, %d]",
                    target,
                    robot.x_axis->get_min_position(),
                    robot.x_axis->get_max_position()
                );
                return hardware_interface::return_type::ERROR;
            }

            // Check for existing errors before commanding
            if (robot.x_axis->get_error())
            {
                RCLCPP_WARN(
                    logger,
                    "Cannot move X-axis - axis has error: %d",
                    robot.x_axis->get_error_id()
                );
                return hardware_interface::return_type::ERROR;
            }

            // Send command with error handling
            try
            {
                robot.x_axis->move(target);
                RCLCPP_DEBUG(logger, "X-axis move command sent to %d", target);
            }
            catch (const PMClient::OpcuaException &e)
            {
                RCLCPP_ERROR(logger, "Failed to send move command to X-axis: %s", e.what());
                return hardware_interface::return_type::ERROR;
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error in write(): %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }
};

// ==============================================================================
// PATTERN 2: STANDALONE ROS2 NODE WITH PM CLIENT
// ==============================================================================

class RobotControlNode : public rclcpp::Node
{
  private:
    std::unique_ptr<PMClient::Client> m_client;
    rclcpp::TimerBase::SharedPtr m_timer;

  public:
    RobotControlNode() : rclcpp::Node("robot_control")
    {
        // Declare parameters
        this->declare_parameter<std::string>("pm_client_log_level", "warning");
        this->declare_parameter<std::string>("opcua_endpoint", "opc.tcp://localhost:4840");

        // Get parameters
        std::string log_level = this->get_parameter("pm_client_log_level").as_string();
        std::string endpoint = this->get_parameter("opcua_endpoint").as_string();

        // Configure logging
        if (log_level == "debug")
        {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Debug);
        }
        else if (log_level == "info")
        {
            PMClient::DebugLogger::set_level(PMClient::LogLevel::Info);
        }

        // Initialize PM Client
        try
        {
            m_client = std::make_unique<PMClient::Client>();
            m_client->connect(endpoint);
            m_client->init();
            RCLCPP_INFO(get_logger(), "PM Client initialized successfully");
        }
        catch (const PMClient::OpcuaException &e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize PM Client: %s", e.what());
            throw;
        }

        // Create a timer for main control loop
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotControlNode::control_loop, this)
        );

        // Create services
        auto move_service = this->create_service<pm_msgs::srv::MoveAxis>(
            "move_axis",
            std::bind(
                &RobotControlNode::handle_move_axis,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
    }

  private:
    void control_loop()
    {
        try
        {
            auto &robot = m_client->get_robot();

            // Read current state with error handling
            if (robot.x_axis && robot.x_axis->is_ok())
            {
                try
                {
                    int pos = robot.x_axis->get_actual_position();
                    RCLCPP_DEBUG(get_logger(), "X-axis position: %d", pos);
                }
                catch (const PMClient::OpcuaException &e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to read X-axis position: %s", e.what());
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Unexpected error in control loop: %s", e.what());
        }
    }

    void handle_move_axis(
        const std::shared_ptr<pm_msgs::srv::MoveAxis::Request> request,
        const std::shared_ptr<pm_msgs::srv::MoveAxis::Response> response
    )
    {
        try
        {
            auto &robot = m_client->get_robot();

            // Validate axis ID
            PMClient::AerotechAxis *axis = nullptr;
            switch (request->axis)
            {
                case 0:
                    axis = robot.x_axis.get();
                    break;
                case 1:
                    axis = robot.y_axis.get();
                    break;
                case 2:
                    axis = robot.z_axis.get();
                    break;
                default:
                    response->success = false;
                    response->message = "Invalid axis ID";
                    RCLCPP_WARN(get_logger(), "Invalid axis ID: %d", request->axis);
                    return;
            }

            // Check axis is initialized
            if (!axis || !axis->is_ok())
            {
                response->success = false;
                response->message = "Axis not initialized";
                RCLCPP_ERROR(get_logger(), "Axis %d not properly initialized", request->axis);
                return;
            }

            // Check for existing errors
            if (axis->get_error())
            {
                response->success = false;
                response->message = "Axis has error: " + std::to_string(axis->get_error_id());
                RCLCPP_WARN(
                    get_logger(),
                    "Axis %d error ID: %d",
                    request->axis,
                    axis->get_error_id()
                );
                return;
            }

            // Validate target position
            int target = request->target;
            if (target < axis->get_min_position() || target > axis->get_max_position())
            {
                response->success = false;
                response->message = "Target out of range";
                RCLCPP_WARN(
                    get_logger(),
                    "Target %d out of range [%d, %d]",
                    target,
                    axis->get_min_position(),
                    axis->get_max_position()
                );
                return;
            }

            // Send move command
            try
            {
                axis->move(target);
                response->success = true;
                response->message = "Move command sent";
                RCLCPP_INFO(get_logger(), "Axis %d move to %d sent", request->axis, target);
            }
            catch (const PMClient::OpcuaException &e)
            {
                response->success = false;
                response->message = std::string("OPC UA error: ") + e.what();
                RCLCPP_ERROR(get_logger(), "Failed to move axis %d: %s", request->axis, e.what());
            }
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = std::string("Unexpected error: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Unexpected error in move_axis service: %s", e.what());
        }
    }
};

// ==============================================================================
// PATTERN 3: WRAPPING PM CLIENT IN HELPER CLASS FOR CONSISTENCY
// ==============================================================================

/**
 * Wrapper around pm_client that centralizes error handling
 * and logging for easier integration into multiple components
 */
class PMClientWrapper
{
  private:
    std::unique_ptr<PMClient::Client> m_client;
    std::string m_logger_name;

  public:
    explicit PMClientWrapper(const std::string &logger_name) : m_logger_name(logger_name)
    {
    }

    bool initialize(const std::string &endpoint)
    {
        auto logger = rclcpp::get_logger(m_logger_name);

        try
        {
            m_client = std::make_unique<PMClient::Client>();

            RCLCPP_INFO(logger, "Connecting to OPC UA server: %s", endpoint.c_str());
            m_client->connect(endpoint);

            RCLCPP_INFO(logger, "Initializing PM Client...");
            m_client->init();

            // Validate components
            auto &robot = m_client->get_robot();

            std::vector<std::string> missing_components;
            if (!robot.x_axis || !robot.x_axis->is_ok())
                missing_components.push_back("X-axis");
            if (!robot.y_axis || !robot.y_axis->is_ok())
                missing_components.push_back("Y-axis");
            if (!robot.z_axis || !robot.z_axis->is_ok())
                missing_components.push_back("Z-axis");

            if (!missing_components.empty())
            {
                std::string missing_str;
                for (size_t i = 0; i < missing_components.size(); ++i)
                {
                    if (i > 0)
                        missing_str += ", ";
                    missing_str += missing_components[i];
                }
                RCLCPP_ERROR(logger, "Missing or invalid components: %s", missing_str.c_str());
                return false;
            }

            RCLCPP_INFO(logger, "PM Client ready");
            return true;
        }
        catch (const PMClient::OpcuaException &e)
        {
            RCLCPP_ERROR(logger, "OPC UA error: %s", e.what());
            return false;
        }
        catch (const PMClient::InitializationException &e)
        {
            RCLCPP_ERROR(logger, "Initialization error: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error: %s", e.what());
            return false;
        }
    }

    bool safe_move_axis(PMClient::AxisId axis_id, int target)
    {
        auto logger = rclcpp::get_logger(m_logger_name);

        try
        {
            if (!m_client)
            {
                RCLCPP_ERROR(logger, "PM Client not initialized");
                return false;
            }

            auto &robot = m_client->get_robot();
            PMClient::AerotechAxis *axis = get_axis(robot, axis_id);

            if (!axis)
            {
                RCLCPP_ERROR(logger, "Cannot get axis");
                return false;
            }

            if (!axis->is_ok())
            {
                RCLCPP_ERROR(logger, "Axis not properly initialized");
                return false;
            }

            if (axis->get_error())
            {
                RCLCPP_WARN(logger, "Axis has error %d", axis->get_error_id());
                return false;
            }

            if (target < axis->get_min_position() || target > axis->get_max_position())
            {
                RCLCPP_ERROR(logger, "Target %d out of range", target);
                return false;
            }

            axis->move(target);
            RCLCPP_DEBUG(logger, "Move command sent successfully");
            return true;
        }
        catch (const PMClient::OpcuaException &e)
        {
            RCLCPP_ERROR(logger, "OPC UA error: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Unexpected error: %s", e.what());
            return false;
        }
    }

    PMClient::Client *get_client()
    {
        return m_client.get();
    }

  private:
    PMClient::AerotechAxis *get_axis(PMClient::Robot &robot, PMClient::AxisId axis_id)
    {
        switch (axis_id)
        {
            case PMClient::AxisId::X:
                return robot.x_axis.get();
            case PMClient::AxisId::Y:
                return robot.y_axis.get();
            case PMClient::AxisId::Z:
                return robot.z_axis.get();
            case PMClient::AxisId::T:
                return robot.t_axis.get();
            case PMClient::AxisId::Q:
                return robot.q_axis.get();
            case PMClient::AxisId::R:
                return robot.r_axis.get();
            case PMClient::AxisId::U:
                return robot.u_axis.get();
            case PMClient::AxisId::V:
                return robot.v_axis.get();
            default:
                return nullptr;
        }
    }
};

} // namespace pm_ros2_integration
