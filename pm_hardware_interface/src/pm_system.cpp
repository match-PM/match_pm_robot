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
    catch (std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMSystem"),
            "Failed to initialize PMClient (%s).",
            e.what()
        );
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

    auto &robot = m_pm_client.get_robot();

    for (auto &axis : m_axes)
    {
        axis.read(robot);
    }

    for (auto &pneumatic : m_pneumatics)
    {
        pneumatic.read(robot);
    }

    for (auto &nozzle : m_nozzles)
    {
        nozzle.read(robot);
    }

    m_camera1_coax_light = static_cast<double>(robot.camera1->get_coax_light());

    bool segments[4] = {0};
    robot.camera1->get_ring_light(segments[0], segments[1], segments[2], segments[3]);
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light[i] = static_cast<double>(segments[i]);
    }

    int rgb[3] = {0};
    robot.camera1->get_ring_light_color(rgb[0], rgb[1], rgb[2]);
    for (std::size_t i = 0; i < 3; i++)
    {
        m_camera1_ring_light_rgb[i] = static_cast<double>(rgb[i]);
    }

    m_camera2_light = static_cast<double>(robot.camera2->get_light());

    m_laser_measurement = robot.laser->get_measurement();

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

    for (auto &axis : m_axes)
    {
        axis.add_state_interfaces(state_interfaces);
    }

    for (auto &pneumatic : m_pneumatics)
    {
        pneumatic.add_state_interfaces(state_interfaces);
    }

    for (auto &nozzle : m_nozzles)
    {
        nozzle.add_state_interfaces(state_interfaces);
    }

    state_interfaces.emplace_back(
        StateInterface("Camera1_Coax_Light", "On_Off", &m_camera1_coax_light)
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "1_On_Off", &m_camera1_ring_light[0])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "2_On_Off", &m_camera1_ring_light[1])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "3_On_Off", &m_camera1_ring_light[2])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "4_On_Off", &m_camera1_ring_light[3])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Red", &m_camera1_ring_light_rgb[0])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Green", &m_camera1_ring_light_rgb[1])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Blue", &m_camera1_ring_light_rgb[2])
    );

    state_interfaces.emplace_back(StateInterface("Camera2_Light", "Intensity", &m_camera2_light));

    state_interfaces.emplace_back(StateInterface("Laser", "Measurement", &m_laser_measurement));

    return state_interfaces;
}

std::vector<CommandInterface> PMSystem::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;

    for (auto &axis : m_axes)
    {
        axis.add_command_interfaces(command_interfaces);
    }

    for (auto &pneumatic : m_pneumatics)
    {
        pneumatic.add_command_interfaces(command_interfaces);
    }

    for (auto &nozzle : m_nozzles)
    {
        nozzle.add_command_interfaces(command_interfaces);
    }

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Coax_Light", "On_Off", &m_camera1_coax_light)
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "1_On_Off", &m_camera1_ring_light[0])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "2_On_Off", &m_camera1_ring_light[1])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "3_On_Off", &m_camera1_ring_light[2])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "4_On_Off", &m_camera1_ring_light[3])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Red", &m_camera1_ring_light_rgb[0])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Green", &m_camera1_ring_light_rgb[1])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Blue", &m_camera1_ring_light_rgb[2])
    );

    command_interfaces.emplace_back(CommandInterface("Camera2_Light", "Intensity", &m_camera2_light)
    );

    return command_interfaces;
}

hardware_interface::return_type
PMSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    auto &robot = m_pm_client.get_robot();

    // RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem::read called.");

    for (auto &axis : m_axes)
    {
        axis.read(robot);
    }

    for (auto &pneumatic : m_pneumatics)
    {
        pneumatic.read(robot);
    }

    for (auto &nozzle : m_nozzles)
    {
        nozzle.read(robot);
    }

    m_camera1_coax_light = static_cast<double>(robot.camera1->get_coax_light());

    bool segments[4] = {0};
    robot.camera1->get_ring_light(segments[0], segments[1], segments[2], segments[3]);
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light[i] = static_cast<double>(segments[i]);
    }

    int rgb[3] = {0};
    robot.camera1->get_ring_light_color(rgb[0], rgb[1], rgb[2]);
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light_rgb[i] = static_cast<double>(rgb[i]);
    }

    m_camera2_light = static_cast<double>(robot.camera2->get_light());

    m_laser_measurement = robot.laser->get_measurement();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PMSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    // RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem::write called.");

    auto &robot = m_pm_client.get_robot();

    for (auto &axis : m_axes)
    {
        axis.write(robot);
    }

    for (auto &pneumatic : m_pneumatics)
    {
        pneumatic.write(robot);
    }

    for (auto &nozzle : m_nozzles)
    {
        nozzle.write(robot);
    }

    robot.camera1->set_coax_light(static_cast<bool>(m_camera1_coax_light));

    bool segments[4] = {0};
    for (std::size_t i = 0; i < 4; i++)
    {
        segments[i] = static_cast<bool>(m_camera1_ring_light[i]);
    }
    robot.camera1->set_ring_light(segments[0], segments[1], segments[2], segments[3]);

    int rgb[3] = {0};
    for (std::size_t i = 0; i < 3; i++)
    {
        rgb[i] = static_cast<int>(m_camera1_ring_light_rgb[i]);
    }
    robot.camera1->set_ring_light_color(rgb[0], rgb[1], rgb[2]);

    robot.camera2->set_light(static_cast<int>(m_camera2_light));

    return hardware_interface::return_type::OK;
}

//
// ---------------------------------------------
//
} // namespace pm_hardware_interface

PLUGINLIB_EXPORT_CLASS(pm_hardware_interface::PMSystem, hardware_interface::SystemInterface)
