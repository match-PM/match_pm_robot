#ifndef PM_GRIPPER_INTERFACE_H
#define PM_GRIPPER_INTERFACE_H

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"

#include "visibility_control.h"

#include "MCS2/SmarActControl.h"

namespace pm_smaract_gripper_interface
{

using State = rclcpp_lifecycle::State;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using HardwareInfo = hardware_interface::HardwareInfo;

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using double_limits = std::numeric_limits<double>;

class PMGripperInterface : public hardware_interface::SystemInterface
{
    constexpr static int MAX_READ_ATTEMPTS = 5;

    SA_CTL_DeviceHandle_t m_handle;
    int8_t m_channel = 0;

    int32_t m_current_move_mode = -1;

    double m_rel_target_position{double_limits::quiet_NaN()};
    double m_abs_target_position{double_limits::quiet_NaN()};
    double m_target_velocity{double_limits::quiet_NaN()};
    double m_target_acceleration{double_limits::quiet_NaN()};
    double m_current_velocity{double_limits::quiet_NaN()};
    double m_current_acceleration{double_limits::quiet_NaN()};

    double m_current_position{0.0};
    struct
    {
        double x;
        double y;
        double z;
    } m_force{};

  public:
    PMGripperInterface();

    RCLCPP_SHARED_PTR_DEFINITIONS(PMGripperInterface)

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

    ///
    /// PMGripperInterface methods
    ///

    void set_move_mode(int32_t move_mode);

    SA_CTL_Result_t
    try_read_property(int8_t channel, SA_CTL_PropertyKey_t property, double &out_value);

    //
    // ---------------------------------------------
    //
};

} // namespace pm_smaract_gripper_interface

#endif // PM_GRIPPER_INTERFACE_H
