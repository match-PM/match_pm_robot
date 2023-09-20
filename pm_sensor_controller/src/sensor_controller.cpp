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

    m_laser_pub = get_node()->create_publisher<std_msgs::msg::Float64>("~/laser", 1000);
    m_force_pub = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/force", 1000);
    m_force_bias_sub = get_node()->create_subscription<std_msgs::msg::Bool>(
        "~/force_bias",
        rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { m_force_sensor_bias_cmd = msg->data; }
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
    };
    return config;
}

controller_interface::return_type
PMSensorController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    {
        std_msgs::msg::Float64 msg;
        msg.data = state_interfaces_[0].get_value();
        m_laser_pub->publish(msg);
    }

    {
        std_msgs::msg::Float64MultiArray msg;
        for (auto i = 0; i < 6; i++)
        {
            msg.data.emplace_back(state_interfaces_[1 + i].get_value());
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
