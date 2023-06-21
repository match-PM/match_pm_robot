#ifndef PM_SYSTEM_H
#define PM_SYSTEM_H

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"

#include "pm_client/client.hpp"

#include "visibility_control.h"

namespace pm_hardware_interface
{

using State = rclcpp_lifecycle::State;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using HardwareInfo = hardware_interface::HardwareInfo;

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

class PMSystem : public hardware_interface::SystemInterface
{
    struct Config
    {
        std::string opcua_endpoint;
    };

    struct AxisState
    {
        double current_position = 0.0;
        double target_position = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;
    };

    PMClient::Client m_pm_client;
    Config m_config;

    AxisState m_x_axis, m_y_axis, m_z_axis, m_t_axis;

    double m_camera1_coax_light;
    double m_camera1_ring_light[4];
    double m_camera1_ring_light_rgb[3];
    double m_camera2_light;

  public:
    PMSystem();

    RCLCPP_SHARED_PTR_DEFINITIONS(PMSystem)

    //
    // LifecycleNodeInterface methods
    //

    PM_SYSTEM_PUBLIC CallbackReturn on_configure(const State &previous_state) override;

    PM_SYSTEM_PUBLIC CallbackReturn on_cleanup(const State &previous_state) override;

    PM_SYSTEM_PUBLIC CallbackReturn on_shutdown(const State &previous_state) override;

    PM_SYSTEM_PUBLIC CallbackReturn on_activate(const State &previous_state) override;

    PM_SYSTEM_PUBLIC CallbackReturn on_deactivate(const State &previous_state) override;

    PM_SYSTEM_PUBLIC CallbackReturn on_error(const State &previous_state) override;

    //
    // ---------------------------------------------
    //

    //
    // SystemInterface methods
    //

    PM_SYSTEM_PUBLIC CallbackReturn on_init(const HardwareInfo &hardware_info) override;

    PM_SYSTEM_PUBLIC std::vector<StateInterface> export_state_interfaces() override;

    PM_SYSTEM_PUBLIC std::vector<CommandInterface> export_command_interfaces() override;

    PM_SYSTEM_PUBLIC hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    PM_SYSTEM_PUBLIC hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    //
    // ---------------------------------------------
    //
};

} // namespace pm_hardware_interface

#endif // PM_SYSTEM_H
