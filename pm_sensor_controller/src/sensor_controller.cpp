#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

#include "pm_sensor_controller/sensor_controller.hpp"

namespace pm_sensor_controller
{

PMSensorController::PMSensorController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMSensorController"), "PMSensorController instantiated.");
}

controller_interface::return_type PMSensorController::init(
    const std::string &controller_name, const std::string &namespace_,
    const rclcpp::NodeOptions &node_options
)
{
    return controller_interface::ControllerInterface::init(
        controller_name,
        namespace_,
        node_options
    );
}

controller_interface::CallbackReturn PMSensorController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_reference_cube_pub =
        get_node()->create_publisher<std_msgs::msg::Bool>("~/ReferenceCube/Stream", 1000);
    m_laser_pub = get_node()->create_publisher<std_msgs::msg::Float64>("~/Laser/Stream", 1000);
    m_force_pub = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/ForceSensor/Stream",
        1000
    );

    m_force_sensor_get_measurement = get_node()->create_service<ForceSensorGetMeasurement>(
        "~/ForceSensor/GetMeasurement",
        [this](
            const ForceSensorGetMeasurement::Request::SharedPtr request,
            ForceSensorGetMeasurement::Response::SharedPtr response
        ) {
            (void)request;
            std::copy_n(
                std::begin(response->data),
                m_force_sensor_measurement.size(),
                std::begin(m_force_sensor_measurement)
            );
        }
    );

    m_force_sensor_bias = get_node()->create_service<ForceSensorBias>(
        "~/ForceSensor/Bias",
        [this](
            const ForceSensorBias::Request::SharedPtr request,
            ForceSensorBias::Response::SharedPtr response
        ) {
            (void)response;
            (void)request;
            m_force_sensor_bias_cmd = true;
        }
    );

    m_laser_get_measurement = get_node()->create_service<LaserGetMeasurement>(
        "~/Laser/GetMeasurement",
        [this](
            const LaserGetMeasurement::Request::SharedPtr request,
            LaserGetMeasurement::Response::SharedPtr response
        ) {
            (void)request;
            response->measurement = m_laser_measurement;
        }
    );

    m_reference_cube_state = get_node()->create_service<ReferenceCubeState>(
        "~/ReferenceCube/State",
        [this](
            const ReferenceCubeState::Request::SharedPtr request,
            ReferenceCubeState::Response::SharedPtr response
        ) {
            (void)request;
            response->pressed = m_reference_cube_pressed;
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMSensorController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Force/Bias",
    };
    return config;
}

controller_interface::InterfaceConfiguration
PMSensorController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Laser/Measurement",
        "Force/X",
        "Force/Y",
        "Force/Z",
        "Force/TX",
        "Force/TY",
        "Force/TZ",
        "ReferenceCube/Pushed",
    };
    return config;
}

controller_interface::return_type
PMSensorController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    {
        std_msgs::msg::Bool msg;
        m_reference_cube_pressed = msg.data = static_cast<bool>(state_interfaces_[7].get_value());
        m_reference_cube_pub->publish(msg);
    }

    {
        std_msgs::msg::Float64 msg;
        m_laser_measurement = msg.data = state_interfaces_[0].get_value();
        m_laser_pub->publish(msg);
    }

    {
        std_msgs::msg::Float64MultiArray msg;
        for (auto i = 0; i < 6; i++)
        {
            m_force_sensor_measurement[i] = state_interfaces_[1 + i].get_value();
            msg.data.emplace_back(m_force_sensor_measurement[i]);
        }
        m_force_pub->publish(msg);
    }

    command_interfaces_[0].set_value(static_cast<double>(m_force_sensor_bias_cmd));
    m_force_sensor_bias_cmd = false;

    return controller_interface::return_type::OK;
}

} // namespace pm_sensor_controller

PLUGINLIB_EXPORT_CLASS(
    pm_sensor_controller::PMSensorController, controller_interface::ControllerInterface
)
