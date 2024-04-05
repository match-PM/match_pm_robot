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

    m_camera1_coax_light_state = static_cast<double>(robot.camera1->get_coax_light());

    bool segments[4] = {0};
    robot.camera1->get_ring_light(segments[0], segments[1], segments[2], segments[3]);
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light_state[i] = static_cast<double>(segments[i]);
    }

    int rgb[3] = {0};
    robot.camera1->get_ring_light_color(rgb[0], rgb[1], rgb[2]);
    for (std::size_t i = 0; i < 3; i++)
    {
        m_camera1_ring_light_rgb_state[i] = static_cast<double>(rgb[i]);
    }

    m_camera2_light_state = static_cast<double>(robot.camera2->get_light());

    m_laser_measurement = robot.laser->get_measurement();

    hoenle_uv.read(robot);

    reference_cube_pushed = static_cast<double>(robot.reference_cube->get_pushed());

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

    // Dummy state interfaces so that MoveIt doesn't complain
    // about missing joint data
    // TODO: Implement properly
    // State interfaces for these axes were implemented with a mock hardware and a
    // JointgroupController Consider adding the hardware interface for moving the pneumatic axes in
    // here properly

    // std::vector<std::string> dummies{
    //     "Calibration_Qube_Joint",
    //     "Camera_Calibration_Platelet_Joint",
    //     "UV_Slider_X_Back_Joint",
    //     "UV_LED_Back_Joint",
    //     "UV_Slider_X_Front_Joint",
    //     "UV_LED_Front_Joint",
    //     "1K_Dispenser_Flap_Joint",
    //     "1K_Dispenser_Joint",
    //     "2K_Dispenser_Joint"
    // };

    std::vector<std::string> dummies{
        "Calibration_Qube_Joint",
        "UV_Slider_X_Back_Joint",
        "UV_Slider_X_Front_Joint",
    };
    for (auto &dummy : dummies)
    {
        state_interfaces.emplace_back(
            StateInterface(dummy, hardware_interface::HW_IF_POSITION, &m_camera1_coax_light_state)
        );
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
        StateInterface("Camera1_Coax_Light", "On_Off", &m_camera1_coax_light_state)
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "1_On_Off", &m_camera1_ring_light_state[0])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "2_On_Off", &m_camera1_ring_light_state[1])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "3_On_Off", &m_camera1_ring_light_state[2])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "4_On_Off", &m_camera1_ring_light_state[3])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Red", &m_camera1_ring_light_rgb_state[0])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Green", &m_camera1_ring_light_rgb_state[1])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera1_Ring_Light", "Blue", &m_camera1_ring_light_rgb_state[2])
    );

    state_interfaces.emplace_back(
        StateInterface("Camera2_Light", "Intensity", &m_camera2_light_state)
    );

    state_interfaces.emplace_back(StateInterface("Laser", "Measurement", &m_laser_measurement));

    state_interfaces.emplace_back(StateInterface("Force", "X", &m_force_sensor_measurements[0]));
    state_interfaces.emplace_back(StateInterface("Force", "Y", &m_force_sensor_measurements[1]));
    state_interfaces.emplace_back(StateInterface("Force", "Z", &m_force_sensor_measurements[2]));
    state_interfaces.emplace_back(StateInterface("Force", "TX", &m_force_sensor_measurements[3]));
    state_interfaces.emplace_back(StateInterface("Force", "TY", &m_force_sensor_measurements[4]));
    state_interfaces.emplace_back(StateInterface("Force", "TZ", &m_force_sensor_measurements[5]));

    hoenle_uv.add_state_interfaces(state_interfaces);

    state_interfaces.emplace_back(StateInterface("ReferenceCube", "Pushed", &reference_cube_pushed)
    );

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
        CommandInterface("Camera1_Coax_Light", "On_Off", &m_camera1_coax_light_cmd)
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "1_On_Off", &m_camera1_ring_light_cmd[0])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "2_On_Off", &m_camera1_ring_light_cmd[1])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "3_On_Off", &m_camera1_ring_light_cmd[2])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "4_On_Off", &m_camera1_ring_light_cmd[3])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Red", &m_camera1_ring_light_rgb_cmd[0])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Green", &m_camera1_ring_light_rgb_cmd[1])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera1_Ring_Light", "Blue", &m_camera1_ring_light_rgb_cmd[2])
    );

    command_interfaces.emplace_back(
        CommandInterface("Camera2_Light", "Intensity", &m_camera2_light_cmd)
    );

    command_interfaces.emplace_back(CommandInterface("Force", "Bias", &m_force_sensor_bias));

    hoenle_uv.add_command_interfaces(command_interfaces);

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

    m_camera1_coax_light_state = static_cast<double>(robot.camera1->get_coax_light());

    bool segments[4] = {0};
    robot.camera1->get_ring_light(segments[0], segments[1], segments[2], segments[3]);
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light_state[i] = static_cast<double>(segments[i]);
    }

    int rgb[3] = {0};
    robot.camera1->get_ring_light_color(rgb[0], rgb[1], rgb[2]);
    for (std::size_t i = 0; i < 3; i++)
    {
        m_camera1_ring_light_rgb_state[i] = static_cast<double>(rgb[i]);
    }

    m_camera2_light_state = static_cast<double>(robot.camera2->get_light());

    m_laser_measurement = robot.laser->get_measurement();

    m_force_sensor_measurements = robot.force_sensor->get_measurements();

    hoenle_uv.read(robot);

    reference_cube_pushed = static_cast<double>(robot.reference_cube->get_pushed());

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

    robot.camera1->set_coax_light(static_cast<bool>(m_camera1_coax_light_cmd));

    bool segments[4] = {0};
    for (std::size_t i = 0; i < 4; i++)
    {
        segments[i] = static_cast<bool>(m_camera1_ring_light_cmd[i]);
    }
    robot.camera1->set_ring_light(segments[0], segments[1], segments[2], segments[3]);

    int rgb[3] = {0};
    for (std::size_t i = 0; i < 3; i++)
    {
        rgb[i] = static_cast<int>(m_camera1_ring_light_rgb_cmd[i]);
    }
    robot.camera1->set_ring_light_color(rgb[0], rgb[1], rgb[2]);

    robot.camera2->set_light(static_cast<int>(m_camera2_light_cmd));

    if (m_force_sensor_bias)
    {
        robot.force_sensor->set_bias();
        m_force_sensor_bias = 0.0;
    }

    hoenle_uv.write(robot);

    return hardware_interface::return_type::OK;
}

//
// ---------------------------------------------
//
} // namespace pm_hardware_interface

PLUGINLIB_EXPORT_CLASS(pm_hardware_interface::PMSystem, hardware_interface::SystemInterface)
