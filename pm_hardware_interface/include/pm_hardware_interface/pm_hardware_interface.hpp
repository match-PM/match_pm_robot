#ifndef PM_HARDWARE_INTERFACE_H
#define PM_HARDWARE_INTERFACE_H

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

#include "pm_client/client.hpp"

namespace pm_hardware_interface
{

using State = rclcpp_lifecycle::State;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using HardwareInfo = hardware_interface::HardwareInfo;

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

class PMSystem : public hardware_interface::SystemInterface
{

    PMClient::Client m_pm_client;

  public:
    PMSystem();

    //
    // LifecycleNodeInterface methods
    //

    CallbackReturn on_configure(const State &previous_state) override;

    CallbackReturn on_cleanup(const State &previous_state) override;

    CallbackReturn on_shutdown(const State &previous_state) override;

    CallbackReturn on_activate(const State &previous_state) override;

    CallbackReturn on_deactivate(const State &previous_state) override;

    CallbackReturn on_error(const State &previous_state) override;

    //
    // ---------------------------------------------
    //

    //
    // SystemInterface methods
    //

    CallbackReturn on_init(const HardwareInfo &hardware_info) override;

    std::vector<StateInterface> export_state_interfaces() override;

    std::vector<CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    //
    // ---------------------------------------------
    //
};

} // namespace pm_hardware_interface

#endif // PM_HARDWARE_INTERFACE_H
