#include "pluginlib/class_list_macros.hpp"

#include "pm_pneumatic_controller/pneumatic_controller.hpp"

namespace pm_pneumatic_controller
{

PMPneumaticController::PMPneumaticController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMPneumaticController"), "PMPneumaticController instantiated.");
}

controller_interface::return_type PMPneumaticController::init(
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

controller_interface::CallbackReturn PMPneumaticController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    m_cylinder_sub = get_node()->create_subscription<PneumaticCylinderCmd>(
        "~/cylinder",
        rclcpp::SystemDefaultsQoS(),
        [this](const PneumaticCylinderCmd::SharedPtr msg) {
            m_cylinder_move_forward_cmd = msg->move_forward;
        }
    );
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    m_cylinder_move_forward_cmd = static_cast<bool>(state_interfaces_[0].get_value());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "UV1_Pneumatic/Move_Forward",
        "UV1_Pneumatic/Move_Backward",
    };
    return config;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "UV1_Pneumatic/Is_Forward",
        "UV1_Pneumatic/Is_Backward",
    };
    return config;
}

controller_interface::return_type
PMPneumaticController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    command_interfaces_[0].set_value(static_cast<double>(m_cylinder_move_forward_cmd));
    command_interfaces_[1].set_value(static_cast<double>(!m_cylinder_move_forward_cmd));
    return controller_interface::return_type::OK;
}

} // namespace pm_pneumatic_controller

PLUGINLIB_EXPORT_CLASS(
    pm_pneumatic_controller::PMPneumaticController, controller_interface::ControllerInterface
)
