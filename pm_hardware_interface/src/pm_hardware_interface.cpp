#include <fstream>

#include "pluginlib/class_list_macros.hpp"
#include "pm_hardware_interface/pm_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pm_hardware_interface
{

PMSystem::PMSystem()
{
    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem instantiated.");
}

//
// LifecycleNodeInterface methods
//

CallbackReturn PMSystem::on_configure(const State &previous_state)
{
    (void)previous_state;

    m_pm_client.init();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_cleanup(const State &previous_state)
{
    (void)previous_state;

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_shutdown(const State &previous_state)
{
    (void)previous_state;

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_activate(const State &previous_state)
{
    (void)previous_state;

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_deactivate(const State &previous_state)
{
    (void)previous_state;

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_error(const State &previous_state)
{
    (void)previous_state;

    return CallbackReturn::FAILURE;
}

//
// ---------------------------------------------
//

//
// SystemInterface methods
//

CallbackReturn PMSystem::on_init(const HardwareInfo &hardware_info)
{
    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Hardware Info Joints:");
    for (const auto &joint : hardware_info.joints)
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "  - %s", joint.name.c_str());
    }

    CallbackReturn ret = hardware_interface::SystemInterface::on_init(hardware_info);
    if (ret != CallbackReturn::SUCCESS)
    {
        return ret;
    }

    return CallbackReturn::FAILURE;

    m_pm_client.connect("");

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PMSystem::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;

    return state_interfaces;
}

std::vector<CommandInterface> PMSystem::export_command_interfaces()
{
    return {};
}

hardware_interface::return_type
PMSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PMSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    return hardware_interface::return_type::OK;
}

//
// ---------------------------------------------
//
} // namespace pm_hardware_interface

PLUGINLIB_EXPORT_CLASS(pm_hardware_interface::PMSystem, hardware_interface::SystemInterface)
