#include "pluginlib/class_list_macros.hpp"

#include "pm_laser_controller/laser_controller.hpp"

namespace pm_laser_controller
{

PMLaserController::PMLaserController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMLaserController"), "PMLaserController instantiated.");
}

controller_interface::return_type PMLaserController::init(
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

controller_interface::CallbackReturn PMLaserController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLaserController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_measurement_pub = get_node()->create_publisher<std_msgs::msg::Float64>("~/measurement", 1000);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLaserController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLaserController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMLaserController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

controller_interface::InterfaceConfiguration
PMLaserController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Laser/Measurement",
    };
    return config;
}

controller_interface::return_type
PMLaserController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    std_msgs::msg::Float64 msg;
    msg.data = state_interfaces_[0].get_value();
    m_measurement_pub->publish(msg);

    return controller_interface::return_type::OK;
}

} // namespace pm_laser_controller

PLUGINLIB_EXPORT_CLASS(
    pm_laser_controller::PMLaserController, controller_interface::ControllerInterface
)
