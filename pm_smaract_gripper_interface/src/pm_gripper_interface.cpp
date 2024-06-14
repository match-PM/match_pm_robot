#include <fstream>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "pm_smaract_gripper_interface/pm_gripper_interface.hpp"

#include "MCS2/SmarActControl.h"

namespace pm_smaract_gripper_interface
{

PMGripperInterface::PMGripperInterface()
{
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "PMGripperInterface instantiated.");
}

//
// LifecycleNodeInterface methods
//

CallbackReturn PMGripperInterface::on_configure(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Configuring PMGripperInterface...");

    char device_list[1024]{};
    size_t device_list_len = sizeof(device_list);
    auto result = SA_CTL_FindDevices("", device_list, &device_list_len);
    if (result != SA_CTL_ERROR_NONE || strlen(device_list) == 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: No devices found."
        );
        return CallbackReturn::FAILURE;
    }

    char *ptr = nullptr;
    strtok_r(device_list, "\n", &ptr);
    char *first_device = device_list;
    result = SA_CTL_Open(&m_handle, first_device, "");
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could not open device."
        );
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Opened device: %s", first_device);

    int32_t type;
    result = SA_CTL_GetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_CHANNEL_TYPE, &type, 0);
    assert(
        (type == SA_CTL_STICK_SLIP_PIEZO_DRIVER) &&
        "Gripper has unexpected channel type (not slip piezo driver)."
    );

    if (type != SA_CTL_STICK_SLIP_PIEZO_DRIVER)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Unexpected device type."
        );
        return CallbackReturn::FAILURE;
    }

    result = SA_CTL_SetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_MAX_CL_FREQUENCY, 6000);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could not set max CL frequency."
        );
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Set CL frequency: 6000");

    result = SA_CTL_SetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_HOLD_TIME, 1000);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could not set hold time."
        );
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Set hold time: 1000");

    result = SA_CTL_SetProperty_i64(m_handle, m_channel, SA_CTL_PKEY_MOVE_VELOCITY, 1000000000);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could beginnings of gripper hardware interfacenot set move "
            "velocity."
        );
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Set move velocity: 1 mm/s");

    result =
        SA_CTL_SetProperty_i64(m_handle, m_channel, SA_CTL_PKEY_MOVE_ACCELERATION, 10000000000);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could not set move acceleration."
        );
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Set move acceleration: 10 mm/s^2");

    result = SA_CTL_SetProperty_i32(
        m_handle,
        m_channel,
        SA_CTL_PKEY_MOVE_MODE,
        SA_CTL_MOVE_MODE_CL_ABSOLUTE
    );
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to configure: Could not set move mode."
        );
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Set move mode: SA_CTL_MOVE_MODE_CL_ABSOLUTE"
    );

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully configured PMGripperInterface."
    );

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMGripperInterface::on_cleanup(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Cleaning up PMGripperInterface...");

    auto result = SA_CTL_Close(m_handle);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to clean up properly: Could not close device."
        );
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully cleaned up PMGripperInterface."
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMGripperInterface::on_shutdown(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Shutting down PMGripperInterface...");

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully shut down PMGripperInterface."
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMGripperInterface::on_activate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Activating PMGripperInterface...");

    int32_t channel_state = 0;
    auto result =
        SA_CTL_GetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_CHANNEL_STATE, &channel_state, 0);
    if (result != SA_CTL_ERROR_NONE)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("PMGripperInterface"),
            "Failed to activate: Could not get channel state."
        );
        return CallbackReturn::FAILURE;
    }

    if ((channel_state & SA_CTL_CH_STATE_BIT_IS_REFERENCED) == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Need to reference...");

        result = SA_CTL_SetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_REFERENCING_OPTIONS, 0);
        if (result != SA_CTL_ERROR_NONE)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("PMGripperInterface"),
                "Failed to activate: Could not set referencing options."
            );
            return CallbackReturn::FAILURE;
        }

        SA_CTL_Reference(m_handle, m_channel, 0);

        do
        {
            result = SA_CTL_GetProperty_i32(
                m_handle,
                m_channel,
                SA_CTL_PKEY_CHANNEL_STATE,
                &channel_state,
                0
            );
            if (result != SA_CTL_ERROR_NONE)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("PMGripperInterface"),
                    "Failed to activate: Could not get channel state."
                );
                return CallbackReturn::FAILURE;
            }
        }
        while ((channel_state & SA_CTL_CH_STATE_BIT_REFERENCING) != 0);

        RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Referencing complete.");
    }

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully activated PMGripperInterface."
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMGripperInterface::on_deactivate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Deactivating PMGripperInterface...");

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully deactivated PMGripperInterface."
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMGripperInterface::on_error(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "PMGripperInterface encountered an error."
    );

    return CallbackReturn::FAILURE;
}

//
// ---------------------------------------------
//

//
// SystemInterface methods
//

CallbackReturn PMGripperInterface::on_init(const HardwareInfo &hardware_info)
{
    RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "Initializing PMGripperInterface...");
    CallbackReturn ret = hardware_interface::SystemInterface::on_init(hardware_info);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("PMGripperInterface"),
            "SystemInterface::on_init returned error."
        );
        return ret;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("PMGripperInterface"),
        "Successfully initialized PMGripperInterface."
    );

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PMGripperInterface::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;

    state_interfaces.emplace_back("Gripper", "Position", &m_current_position);

    state_interfaces.emplace_back("Gripper", "ForceX", &m_force.x);
    state_interfaces.emplace_back("Gripper", "ForceY", &m_force.y);
    state_interfaces.emplace_back("Gripper", "ForceZ", &m_force.z);

    state_interfaces.emplace_back("Gripper", "Velocity", &m_current_velocity);
    state_interfaces.emplace_back("Gripper", "Acceleration", &m_current_acceleration);

    return state_interfaces;
}

std::vector<CommandInterface> PMGripperInterface::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;

    command_interfaces.emplace_back("Gripper", "TargetPositionRel", &m_rel_target_position);
    command_interfaces.emplace_back("Gripper", "TargetPositionAbs", &m_abs_target_position);
    command_interfaces.emplace_back("Gripper", "TargetVelocity", &m_target_velocity);
    command_interfaces.emplace_back("Gripper", "TargetAcceleration", &m_target_acceleration);

    return command_interfaces;
}

hardware_interface::return_type
PMGripperInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    // RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "PMGripperInterface::read called.");

    int64_t position = 0;
    auto result = SA_CTL_GetProperty_i64(m_handle, m_channel, SA_CTL_PKEY_POSITION, &position, 0);
    if (result != SA_CTL_ERROR_NONE)
        RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "result = %d", result);
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read current position.");
    m_current_position = static_cast<double>(position);

    int64_t force_x = 0;
    result =
        SA_CTL_GetProperty_i64(m_handle, 0, SA_CTL_PKEY_AUX_IO_MODULE_INPUT0_VALUE, &force_x, 0);
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read X force (AUX IO channel 0)");
    m_force.x = static_cast<double>(force_x);

    int64_t force_y = 0;
    result =
        SA_CTL_GetProperty_i64(m_handle, 1, SA_CTL_PKEY_AUX_IO_MODULE_INPUT0_VALUE, &force_y, 0);
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read Y force (AUX IO channel 1)");
    m_force.y = static_cast<double>(force_y);

    int64_t force_z = 0;
    result =
        SA_CTL_GetProperty_i64(m_handle, 2, SA_CTL_PKEY_AUX_IO_MODULE_INPUT0_VALUE, &force_z, 0);
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read Z force (AUX IO channel 2)");
    m_force.z = static_cast<double>(force_z);

    int64_t current_velocity = 0;
    result = SA_CTL_GetProperty_i64(
        m_handle,
        m_channel,
        SA_CTL_PKEY_MOVE_VELOCITY,
        &current_velocity,
        0
    );
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read current velocity.");
    m_current_velocity = static_cast<double>(current_velocity);

    int64_t current_acceleration = 0;
    result = SA_CTL_GetProperty_i64(
        m_handle,
        m_channel,
        SA_CTL_PKEY_MOVE_ACCELERATION,
        &current_acceleration,
        0
    );
    assert((result == SA_CTL_ERROR_NONE) && "Failed to read current acceleration.");
    m_current_acceleration = static_cast<double>(current_acceleration);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PMGripperInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    // RCLCPP_INFO(rclcpp::get_logger("PMGripperInterface"), "PMGripperInterface::write called.");

    if (!std::isnan(m_abs_target_position))
    {
        set_move_mode(SA_CTL_MOVE_MODE_CL_ABSOLUTE);
        auto result =
            SA_CTL_Move(m_handle, m_channel, static_cast<int64_t>(m_abs_target_position), 0);
        if (result != SA_CTL_ERROR_NONE)
            RCLCPP_ERROR(rclcpp::get_logger("PMGripperInterface"), "absolute move failed");
        m_abs_target_position = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_rel_target_position))
    {
        set_move_mode(SA_CTL_MOVE_MODE_CL_RELATIVE);
        auto result =
            SA_CTL_Move(m_handle, m_channel, static_cast<int64_t>(m_rel_target_position), 0);
        if (result != SA_CTL_ERROR_NONE)
            RCLCPP_ERROR(rclcpp::get_logger("PMGripperInterface"), "relative move failed");
        m_rel_target_position = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_target_velocity))
    {
        auto result = SA_CTL_SetProperty_i64(
            m_handle,
            m_channel,
            SA_CTL_PKEY_MOVE_VELOCITY,
            static_cast<int64_t>(m_target_velocity)
        );
        if (result != SA_CTL_ERROR_NONE)
            RCLCPP_ERROR(rclcpp::get_logger("PMGripperInterface"), "setting velocity failed");
        m_target_velocity = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_target_acceleration))
    {
        auto result = SA_CTL_SetProperty_i64(
            m_handle,
            m_channel,
            SA_CTL_PKEY_MOVE_ACCELERATION,
            static_cast<int64_t>(m_target_acceleration)
        );
        if (result != SA_CTL_ERROR_NONE)
            RCLCPP_ERROR(rclcpp::get_logger("PMGripperInterface"), "setting acceleration failed");
        m_target_acceleration = double_limits::quiet_NaN();
    }

    return hardware_interface::return_type::OK;
}

//
// ---------------------------------------------
//

///
/// PMGripperInterface methods
///

void PMGripperInterface::set_move_mode(int32_t move_mode)
{
    if (m_current_move_mode != move_mode)
    {
        auto result = SA_CTL_SetProperty_i32(m_handle, m_channel, SA_CTL_PKEY_MOVE_MODE, move_mode);
        assert((result == SA_CTL_ERROR_NONE) && "Failed to set move mode.");
        m_current_move_mode = move_mode;
    }
}

//
// ---------------------------------------------
//
} // namespace pm_smaract_gripper_interface

PLUGINLIB_EXPORT_CLASS(
    pm_smaract_gripper_interface::PMGripperInterface, hardware_interface::SystemInterface
)
