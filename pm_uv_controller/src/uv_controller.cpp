#include "pluginlib/class_list_macros.hpp"

#include "pm_uv_controller/uv_controller.hpp"

namespace pm_uv_controller
{

PMUVController::PMUVController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMUVController"), "PMUVController instantiated.");
}

controller_interface::return_type PMUVController::init(
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

controller_interface::CallbackReturn PMUVController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_on_off_pub = get_node()->create_publisher<UVOnOff>("~/uv_on_off_state", 1000);

    m_on_off_sub = get_node()->create_subscription<UVOnOff>(
        "~/uv_on_off",
        rclcpp::SystemDefaultsQoS(),
        [this](const UVOnOff::SharedPtr msg) {
            std::copy_n(std::begin(msg->on), 4, std::begin(m_on_off_cmd));
        }
    );

    m_power_sub = get_node()->create_subscription<UVPowerCmd>(
        "~/uv_power",
        rclcpp::SystemDefaultsQoS(),
        [this](const UVPowerCmd::SharedPtr msg) {
            std::copy_n(std::begin(msg->power), 4, std::begin(m_power_cmd));
        }
    );

    m_time_sub = get_node()->create_subscription<UVTimeCmd>(
        "~/uv_time",
        rclcpp::SystemDefaultsQoS(),
        [this](const UVTimeCmd::SharedPtr msg) {
            std::copy_n(std::begin(msg->time), 4, std::begin(m_time_cmd));
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PMUVController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/OnOff_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Power_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Time_" + std::to_string(i));
    }
    return config;
}

controller_interface::InterfaceConfiguration PMUVController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/OnOff_" + std::to_string(i));
    }
    return config;
}

controller_interface::return_type
PMUVController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    {
        UVOnOff msg;
        for (auto i = 0; i < 4; i++)
        {
            msg.on[i] = static_cast<bool>(state_interfaces_[i].get_value());
        }
        m_on_off_pub->publish(msg);
    }

    for (auto i = 0; i < 4; i++)
    {
        command_interfaces_[i].set_value(static_cast<double>(m_on_off_cmd[i]));
    }
    for (auto i = 0; i < 4; i++)
    {
        command_interfaces_[4 + i].set_value(static_cast<double>(m_power_cmd[i]));
    }
    for (auto i = 0; i < 4; i++)
    {
        command_interfaces_[8 + i].set_value(static_cast<double>(m_time_cmd[i]));
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_uv_controller

PLUGINLIB_EXPORT_CLASS(pm_uv_controller::PMUVController, controller_interface::ControllerInterface)
