#include <fstream>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "pm_hardware_interface/pm_system.hpp"

namespace pm_hardware_interface
{

static double increments_to_meters(PMClient::AerotechAxis &axis, int increments)
{
    return axis.increments_to_units(increments) / 1e6;
}

static int meters_to_increments(PMClient::AerotechAxis &axis, double units)
{
    return axis.units_to_increments(units * 1e6);
}

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

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Configuring PMSystem...");

    try
    {
        m_pm_client.connect(m_config.opcua_endpoint);
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMSystem"),
            "Connection to OPCUA server failed: %s.",
            e.what()
        );
        return CallbackReturn::ERROR;
    }

    try
    {
        m_pm_client.init();
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PMSystem"), "Failed to initialize PMClient.");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully configured PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_cleanup(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Cleaning up PMSystem...");

    m_pm_client.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully cleaned up PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_shutdown(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Shutting down PMSystem...");

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully shut down PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_activate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Activating PMSystem...");

    auto *robot = m_pm_client.get_robot();
    std::vector<std::pair<PMClient::AerotechAxis *, AxisState &>> axis = {
        {robot->x_axis.get(), m_x_axis},
        {robot->y_axis.get(), m_y_axis},
        {robot->z_axis.get(), m_z_axis},
        {robot->t_axis.get(), m_t_axis},
    };
    for (const auto &[pm_axis, ros_axis] : axis)
    {
        // RCLCPP_INFO(
        //     rclcpp::get_logger("PMSystem"),
        //     "Initial values:\n  Position: %d\n  Target: %d\n  Speed: %d\n  Acceleration: %d\n",
        //     pm_axis->get_position(),
        //     pm_axis->get_target(),
        //     pm_axis->get_speed(),
        //     pm_axis->get_acceleration()
        // );
        ros_axis.current_position = increments_to_meters(*pm_axis, pm_axis->get_position());
        ros_axis.target_position = increments_to_meters(*pm_axis, pm_axis->get_target());
        ros_axis.velocity = increments_to_meters(*pm_axis, pm_axis->get_speed());
        ros_axis.acceleration = increments_to_meters(*pm_axis, pm_axis->get_acceleration());
    }

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully activated PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_deactivate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Deactivating PMSystem...");

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully deactivated PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_error(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem encountered an error.");

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
    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Initializing PMSystem...");
    CallbackReturn ret = hardware_interface::SystemInterface::on_init(hardware_info);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "SystemInterface::on_init returned error.");
        return ret;
    }

    m_config.opcua_endpoint = hardware_info.hardware_parameters.at("opcua_endpoint");

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully initialized PMSystem.");

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PMSystem::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;

    state_interfaces.emplace_back(StateInterface(
        "X_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_x_axis.current_position
    ));

    state_interfaces.emplace_back(
        StateInterface("X_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_x_axis.velocity)
    );

    state_interfaces.emplace_back(StateInterface(
        "Y_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_y_axis.current_position
    ));

    state_interfaces.emplace_back(
        StateInterface("Y_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_y_axis.velocity)
    );

    state_interfaces.emplace_back(StateInterface(
        "Z_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_z_axis.current_position
    ));

    state_interfaces.emplace_back(
        StateInterface("Z_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_z_axis.velocity)
    );

    state_interfaces.emplace_back(StateInterface(
        "T_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_z_axis.current_position
    ));

    state_interfaces.emplace_back(
        StateInterface("T_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_z_axis.velocity)
    );

    return state_interfaces;
}

std::vector<CommandInterface> PMSystem::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;

    command_interfaces.emplace_back(CommandInterface(
        "X_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_x_axis.target_position
    ));
    command_interfaces.emplace_back(
        CommandInterface("X_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_x_axis.velocity)
    );
    command_interfaces.emplace_back(CommandInterface(
        "X_Axis_Joint",
        hardware_interface::HW_IF_ACCELERATION,
        &m_x_axis.acceleration
    ));

    command_interfaces.emplace_back(CommandInterface(
        "Y_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_y_axis.target_position
    ));
    command_interfaces.emplace_back(
        CommandInterface("Y_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_y_axis.velocity)
    );
    command_interfaces.emplace_back(CommandInterface(
        "Y_Axis_Joint",
        hardware_interface::HW_IF_ACCELERATION,
        &m_y_axis.acceleration
    ));

    command_interfaces.emplace_back(CommandInterface(
        "Z_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_z_axis.target_position
    ));
    command_interfaces.emplace_back(
        CommandInterface("Z_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_z_axis.velocity)
    );
    command_interfaces.emplace_back(CommandInterface(
        "Z_Axis_Joint",
        hardware_interface::HW_IF_ACCELERATION,
        &m_z_axis.acceleration
    ));

    command_interfaces.emplace_back(CommandInterface(
        "T_Axis_Joint",
        hardware_interface::HW_IF_POSITION,
        &m_t_axis.target_position
    ));
    command_interfaces.emplace_back(
        CommandInterface("T_Axis_Joint", hardware_interface::HW_IF_VELOCITY, &m_t_axis.velocity)
    );
    command_interfaces.emplace_back(CommandInterface(
        "T_Axis_Joint",
        hardware_interface::HW_IF_ACCELERATION,
        &m_t_axis.acceleration
    ));

    return command_interfaces;
}

hardware_interface::return_type
PMSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    auto *robot = m_pm_client.get_robot();

    // RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem::read called.");

    std::vector<std::pair<PMClient::AerotechAxis *, AxisState &>> axis = {
        {robot->x_axis.get(), m_x_axis},
        {robot->y_axis.get(), m_y_axis},
        {robot->z_axis.get(), m_z_axis},
        {robot->t_axis.get(), m_t_axis},
    };
    for (const auto &[pm_axis, ros_axis] : axis)
    {
        ros_axis.current_position = increments_to_meters(*pm_axis, pm_axis->get_position());
        ros_axis.velocity = increments_to_meters(*pm_axis, pm_axis->get_speed());
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PMSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    // RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem::write called.");

    auto *robot = m_pm_client.get_robot();
    std::vector<std::pair<PMClient::AerotechAxis *, AxisState &>> axis = {
        {robot->x_axis.get(), m_x_axis},
        // {robot->y_axis.get(), m_y_axis},
        // {robot->z_axis.get(), m_z_axis},
        // {robot->t_axis.get(), m_t_axis},
    };
    for (const auto &[pm_axis, ros_axis] : axis)
    {
        // RCLCPP_INFO(
        //     rclcpp::get_logger("PMSystem"),
        //     "target: %d (%f)",
        //     (meters_to_increments(*pm_axis, ros_axis.target_position)),
        //     ros_axis.target_position
        // );
        pm_axis->move(meters_to_increments(*pm_axis, ros_axis.target_position));
        // pm_axis->set_speed(meters_to_increments(*pm_axis, ros_axis.velocity));
        // pm_axis->set_acceleration(meters_to_increments(*pm_axis, ros_axis.acceleration));
    }

    return hardware_interface::return_type::OK;
}

//
// ---------------------------------------------
//
} // namespace pm_hardware_interface

PLUGINLIB_EXPORT_CLASS(pm_hardware_interface::PMSystem, hardware_interface::SystemInterface)
