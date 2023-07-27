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
    m_param_listener = std::make_shared<ParamListener>(get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_params = m_param_listener->get_params();

    if (m_params.cylinders.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'cylinders' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    m_cylinder_cmds.resize(m_params.cylinders.size(), 0.0);

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        auto subscription = get_node()->create_subscription<PneumaticCylinderCmd>(
            "~/" + m_params.cylinders[i],
            rclcpp::SystemDefaultsQoS(),
            [this, i](const PneumaticCylinderCmd::SharedPtr msg) {
                m_cylinder_cmds[i] = msg->move_forward;
            }
        );
        m_subscriptions.emplace_back(subscription);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        m_cylinder_cmds[i] = static_cast<bool>(state_interfaces_[i].get_value());
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.cylinders)
    {
        config.names.emplace_back(interface + "/Move_Forward");
        config.names.emplace_back(interface + "/Move_Backward");
    }

    return config;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.cylinders)
    {
        config.names.emplace_back(interface + "/Is_Forward");
        config.names.emplace_back(interface + "/Is_Backward");
    }

    return config;
}

controller_interface::return_type
PMPneumaticController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        command_interfaces_[i * 2 + 0].set_value(static_cast<double>(m_cylinder_cmds[i]));
        command_interfaces_[i * 2 + 1].set_value(static_cast<double>(!m_cylinder_cmds[i]));
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_pneumatic_controller

PLUGINLIB_EXPORT_CLASS(
    pm_pneumatic_controller::PMPneumaticController, controller_interface::ControllerInterface
)
