#ifndef PM_SYSTEM_H
#define PM_SYSTEM_H

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"

#include "pm_client/client.hpp"

#include "visibility_control.h"

#include "pm_hardware_interface/axis.hpp"
#include "pm_hardware_interface/hoenle_uv.hpp"
#include "pm_hardware_interface/nozzle.hpp"
#include "pm_hardware_interface/pneumatic.hpp"

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

    PMClient::Client m_pm_client;
    Config m_config;

    std::array<AxisState, 8> m_axes{
        AxisState{AxisId::X, Unit::Meters},
        AxisState{AxisId::Y, Unit::Meters},
        AxisState{AxisId::Z, Unit::Meters},
        AxisState{AxisId::T, Unit::Degrees},
        AxisState{AxisId::Q, Unit::Meters},
        AxisState{AxisId::R, Unit::Meters},
        AxisState{AxisId::U, Unit::Meters},
        AxisState{AxisId::V, Unit::Meters},
    };

    std::array<PneumaticState, 6> m_pneumatics{
        PneumaticState{PneumaticId::UV1},
        PneumaticState{PneumaticId::UV2},
        PneumaticState{PneumaticId::Glue},
        PneumaticState{PneumaticId::Glue2K},
        PneumaticState{PneumaticId::CameraMire},
        PneumaticState{PneumaticId::ProtectDoseur},
    };

    std::array<NozzleState, 5> m_nozzles{
        NozzleState{NozzleId::Head},
        NozzleState{NozzleId::Gonio},
        NozzleState{NozzleId::Nest},
        NozzleState{NozzleId::DoseurGlue},
        NozzleState{NozzleId::DoseurGlue2K},
    };

    HoenleUVState hoenle_uv;

    double m_camera1_coax_light_state{};
    double m_camera1_ring_light_state[4]{};
    double m_camera1_ring_light_rgb_state[3]{};
    double m_camera2_light_state{};

    double m_camera1_coax_light_cmd{};
    double m_camera1_ring_light_cmd[4]{};
    double m_camera1_ring_light_rgb_cmd[3]{};
    double m_camera2_light_cmd{};

    double m_laser_measurement{};

    std::array<double, 7> m_force_sensor_measurements{};
    double m_force_sensor_bias{};

    double reference_cube_pushed{};

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
