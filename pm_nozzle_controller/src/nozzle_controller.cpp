#include "pluginlib/class_list_macros.hpp"

#include "pm_nozzle_controller/nozzle_controller.hpp"

namespace pm_nozzle_controller
{

PMNozzleController::PMNozzleController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMNozzleController"), "PMNozzleController instantiated.");
}

controller_interface::return_type PMNozzleController::init(
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

controller_interface::CallbackReturn PMNozzleController::on_init()
{
    m_param_listener = std::make_shared<ParamListener>(get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMNozzleController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_params = m_param_listener->get_params();

    if (m_params.nozzles.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'nozzles' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    m_nozzle_cmds.resize(m_params.nozzles.size(), 0);

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        auto subscription = get_node()->create_subscription<NozzleCmd>(
            "~/" + m_params.nozzles[i],
            rclcpp::SystemDefaultsQoS(),
            [this, i](const NozzleCmd::SharedPtr msg) { m_nozzle_cmds[i] = msg->cmd; }
        );
        m_subscriptions.emplace_back(subscription);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMNozzleController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        m_nozzle_cmds[i] = static_cast<int>(state_interfaces_[i].get_value());
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMNozzleController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMNozzleController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.nozzles)
    {
        config.names.emplace_back(interface + "/State");
    }

    return config;
}

controller_interface::InterfaceConfiguration
PMNozzleController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.nozzles)
    {
        config.names.emplace_back(interface + "/State");
    }

    return config;
}

controller_interface::return_type
PMNozzleController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        command_interfaces_[i].set_value(static_cast<double>(m_nozzle_cmds[i]));
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_nozzle_controller

PLUGINLIB_EXPORT_CLASS(
    pm_nozzle_controller::PMNozzleController, controller_interface::ControllerInterface
)
