#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
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

PMSystem::~PMSystem()
{
    stop_auxiliary_worker();
}

void PMSystem::start_auxiliary_worker()
{
    std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
    if (m_auxiliary_running)
    {
        return;
    }

    m_auxiliary_running = true;
    m_auxiliary_thread = std::thread(&PMSystem::auxiliary_worker, this);
}

void PMSystem::stop_auxiliary_worker()
{
    {
        std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
        m_auxiliary_running = false;
    }
    m_auxiliary_condition.notify_all();

    if (m_auxiliary_thread.joinable())
    {
        m_auxiliary_thread.join();
    }
}

void PMSystem::auxiliary_worker()
{
    // IO, sensors and lighting have different freshness requirements. They are
    // polled independently and never from the ros2_control callback thread.
    using Clock = std::chrono::steady_clock;
    const auto make_period = [](double rate_hz) {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / rate_hz)
        );
        return std::max(period, std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(1)));
    };

    const auto io_period = make_period(m_config.auxiliary_io_poll_rate_hz);
    const auto sensor_period = make_period(m_config.auxiliary_sensor_poll_rate_hz);
    const auto lighting_period = make_period(m_config.auxiliary_lighting_poll_rate_hz);
    auto next_io_poll = Clock::now() + io_period;
    auto next_sensor_poll = Clock::now() + sensor_period;
    auto next_lighting_poll = Clock::now() + lighting_period;

    unsigned int command_failures = 0;
    unsigned int io_failures = 0;
    unsigned int sensor_failures = 0;
    unsigned int lighting_failures = 0;
    const auto log_failure = [](const char *operation, unsigned int &failure_count) {
        ++failure_count;
        if (failure_count == 1 || failure_count % 50 == 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("PMSystem"),
                "Auxiliary OPC UA %s failed (%u consecutive failures).",
                operation,
                failure_count
            );
        }
    };

    while (true)
    {
        PMClient::AuxiliaryCommands commands{};
        bool poll_io = false;
        bool poll_sensors = false;
        bool poll_lighting = false;

        {
            std::unique_lock<std::mutex> lock(m_auxiliary_mutex);
            auto now = Clock::now();
            auto next_wakeup = std::min(next_io_poll, std::min(next_sensor_poll, next_lighting_poll));
            while (m_auxiliary_running && m_pending_auxiliary_commands.empty() &&
                   now < next_wakeup)
            {
                m_auxiliary_condition.wait_until(lock, next_wakeup);
                now = Clock::now();
                next_wakeup =
                    std::min(next_io_poll, std::min(next_sensor_poll, next_lighting_poll));
            }

            if (!m_auxiliary_running)
            {
                break;
            }

            commands = m_pending_auxiliary_commands;
            m_pending_auxiliary_commands = PMClient::AuxiliaryCommands{};

            now = Clock::now();
            poll_io = now >= next_io_poll;
            poll_sensors = now >= next_sensor_poll;
            poll_lighting = now >= next_lighting_poll;
            if (poll_io)
            {
                next_io_poll = now + io_period;
            }
            if (poll_sensors)
            {
                next_sensor_poll = now + sensor_period;
            }
            if (poll_lighting)
            {
                next_lighting_poll = now + lighting_period;
            }
        }

        try
        {
            if (!commands.empty())
            {
                if (m_auxiliary_client.write_auxiliary_commands(commands))
                {
                    command_failures = 0;
                    std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
                    if (commands.camera1_coax_light_changed)
                    {
                        m_auxiliary_lighting_state.camera1_coax_light =
                            commands.camera1_coax_light;
                    }
                    if (commands.camera1_ring_light_changed)
                    {
                        m_auxiliary_lighting_state.camera1_ring_light =
                            commands.camera1_ring_light;
                    }
                    if (commands.camera1_ring_light_rgb_changed)
                    {
                        m_auxiliary_lighting_state.camera1_ring_light_rgb =
                            commands.camera1_ring_light_rgb;
                    }
                    if (commands.camera2_light_changed)
                    {
                        m_auxiliary_lighting_state.camera2_light = commands.camera2_light;
                    }
                    if (commands.uv_on_off_changed)
                    {
                        m_auxiliary_lighting_state.uv_on_off = commands.uv_on_off;
                    }
                    if (commands.uv_power_changed)
                    {
                        m_auxiliary_lighting_state.uv_power = commands.uv_power;
                    }
                    if (commands.uv_time_changed)
                    {
                        m_auxiliary_lighting_state.uv_time = commands.uv_time;
                    }
                }
                else
                {
                    log_failure("command batch", command_failures);
                }
            }

            if (poll_io)
            {
                PMClient::AuxiliaryIoState state{};
                if (m_auxiliary_client.read_auxiliary_io_state(state))
                {
                    io_failures = 0;
                    std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
                    m_auxiliary_io_state = state;
                }
                else
                {
                    log_failure("IO read", io_failures);
                }
            }

            if (poll_sensors)
            {
                PMClient::AuxiliarySensorState state{};
                if (m_auxiliary_client.read_auxiliary_sensor_state(state))
                {
                    sensor_failures = 0;
                    std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
                    m_auxiliary_sensor_state = state;
                }
                else
                {
                    log_failure("sensor read", sensor_failures);
                }
            }

            if (poll_lighting)
            {
                PMClient::AuxiliaryLightingState state{};
                if (m_auxiliary_client.read_auxiliary_lighting_state(state))
                {
                    lighting_failures = 0;
                    std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
                    m_auxiliary_lighting_state = state;
                }
                else
                {
                    log_failure("lighting/UV read", lighting_failures);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("PMSystem"), "Auxiliary OPC UA worker exception: %s.", e.what()
            );
        }
    }
}

void PMSystem::apply_auxiliary_state()
{
    PMClient::AuxiliaryIoState io_state{};
    PMClient::AuxiliarySensorState sensor_state{};
    PMClient::AuxiliaryLightingState lighting_state{};
    {
        std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
        io_state = m_auxiliary_io_state;
        sensor_state = m_auxiliary_sensor_state;
        lighting_state = m_auxiliary_lighting_state;
    }

    for (std::size_t i = 0; i < m_pneumatics.size(); ++i)
    {
        m_pneumatics[i].update(io_state.pneumatic_positions[i]);
    }
    for (std::size_t i = 0; i < m_nozzles.size(); ++i)
    {
        m_nozzles[i].update(io_state.nozzle_states[i]);
    }
    reference_cube_pushed = static_cast<double>(io_state.reference_cube_pushed);

    m_laser_measurement = sensor_state.laser_measurement;
    m_force_sensor_measurements = sensor_state.force_measurements;

    m_camera1_coax_light_state = static_cast<double>(lighting_state.camera1_coax_light);
    for (std::size_t i = 0; i < 4; ++i)
    {
        m_camera1_ring_light_state[i] =
            static_cast<double>(lighting_state.camera1_ring_light[i]);
        hoenle_uv.on_off_state[i] = static_cast<double>(lighting_state.uv_on_off[i]);
        hoenle_uv.power_state[i] = static_cast<double>(lighting_state.uv_power[i]);
        hoenle_uv.time_state[i] = lighting_state.uv_time[i];
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        m_camera1_ring_light_rgb_state[i] =
            static_cast<double>(lighting_state.camera1_ring_light_rgb[i]);
    }
    m_camera2_light_state = static_cast<double>(lighting_state.camera2_light);
}

void PMSystem::queue_auxiliary_commands()
{
    PMClient::AuxiliaryCommands commands{};

    for (std::size_t i = 0; i < m_pneumatics.size(); ++i)
    {
        const double value = m_pneumatics[i].move_command;
        if (!std::isnan(value))
        {
            commands.pneumatic_changed[i] = true;
            commands.pneumatic_positions[i] = value > 0.0   ? PMClient::Position::Forward
                                                : value < 0.0 ? PMClient::Position::Backward
                                                              : PMClient::Position::Neutral;
            m_pneumatics[i].move_command = std::numeric_limits<double>::quiet_NaN();
        }
    }

    for (std::size_t i = 0; i < m_nozzles.size(); ++i)
    {
        const double value = m_nozzles[i].pressure_cmd;
        if (!std::isnan(value))
        {
            commands.nozzle_changed[i] = true;
            commands.nozzle_states[i] = value > 0.0   ? PMClient::NozzleState::Air
                                         : value < 0.0 ? PMClient::NozzleState::Vacuum
                                                       : PMClient::NozzleState::Off;
            m_nozzles[i].pressure_cmd = std::numeric_limits<double>::quiet_NaN();
        }
    }

    commands.camera1_coax_light = static_cast<bool>(m_camera1_coax_light_cmd);
    commands.camera1_coax_light_changed =
        commands.camera1_coax_light != static_cast<bool>(m_camera1_coax_light_state);

    for (std::size_t i = 0; i < 4; ++i)
    {
        commands.camera1_ring_light[i] = static_cast<bool>(m_camera1_ring_light_cmd[i]);
        commands.camera1_ring_light_changed =
            commands.camera1_ring_light_changed ||
            commands.camera1_ring_light[i] != static_cast<bool>(m_camera1_ring_light_state[i]);
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        commands.camera1_ring_light_rgb[i] =
            static_cast<int>(m_camera1_ring_light_rgb_cmd[i]);
        commands.camera1_ring_light_rgb_changed =
            commands.camera1_ring_light_rgb_changed ||
            commands.camera1_ring_light_rgb[i] !=
                static_cast<int>(m_camera1_ring_light_rgb_state[i]);
    }

    commands.camera2_light = static_cast<int>(m_camera2_light_cmd);
    commands.camera2_light_changed =
        commands.camera2_light != static_cast<int>(m_camera2_light_state);

    if (m_force_sensor_bias != 0.0 && !std::isnan(m_force_sensor_bias))
    {
        commands.force_sensor_bias = true;
        m_force_sensor_bias = 0.0;
    }

    for (std::size_t i = 0; i < 4; ++i)
    {
        if (!std::isnan(hoenle_uv.on_off_cmd[i]))
        {
            commands.uv_on_off_changed = true;
        }
        commands.uv_on_off[i] = std::isnan(hoenle_uv.on_off_cmd[i])
                                    ? static_cast<bool>(hoenle_uv.on_off_state[i])
                                    : static_cast<bool>(hoenle_uv.on_off_cmd[i]);

        if (!std::isnan(hoenle_uv.power_cmd[i]))
        {
            commands.uv_power_changed = true;
        }
        commands.uv_power[i] = std::isnan(hoenle_uv.power_cmd[i])
                                   ? static_cast<int>(hoenle_uv.power_state[i])
                                   : static_cast<int>(hoenle_uv.power_cmd[i]);

        if (!std::isnan(hoenle_uv.time_cmd[i]))
        {
            commands.uv_time_changed = true;
        }
        commands.uv_time[i] =
            std::isnan(hoenle_uv.time_cmd[i]) ? hoenle_uv.time_state[i] : hoenle_uv.time_cmd[i];

        hoenle_uv.on_off_cmd[i] = std::numeric_limits<double>::quiet_NaN();
        hoenle_uv.power_cmd[i] = std::numeric_limits<double>::quiet_NaN();
        hoenle_uv.time_cmd[i] = std::numeric_limits<double>::quiet_NaN();
    }

    if (commands.empty())
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
        for (std::size_t i = 0; i < commands.pneumatic_changed.size(); ++i)
        {
            if (commands.pneumatic_changed[i])
            {
                m_pending_auxiliary_commands.pneumatic_changed[i] = true;
                m_pending_auxiliary_commands.pneumatic_positions[i] =
                    commands.pneumatic_positions[i];
            }
        }
        for (std::size_t i = 0; i < commands.nozzle_changed.size(); ++i)
        {
            if (commands.nozzle_changed[i])
            {
                m_pending_auxiliary_commands.nozzle_changed[i] = true;
                m_pending_auxiliary_commands.nozzle_states[i] = commands.nozzle_states[i];
            }
        }

        if (commands.camera1_coax_light_changed)
        {
            m_pending_auxiliary_commands.camera1_coax_light_changed = true;
            m_pending_auxiliary_commands.camera1_coax_light = commands.camera1_coax_light;
        }
        if (commands.camera1_ring_light_changed)
        {
            m_pending_auxiliary_commands.camera1_ring_light_changed = true;
            m_pending_auxiliary_commands.camera1_ring_light = commands.camera1_ring_light;
        }
        if (commands.camera1_ring_light_rgb_changed)
        {
            m_pending_auxiliary_commands.camera1_ring_light_rgb_changed = true;
            m_pending_auxiliary_commands.camera1_ring_light_rgb = commands.camera1_ring_light_rgb;
        }
        if (commands.camera2_light_changed)
        {
            m_pending_auxiliary_commands.camera2_light_changed = true;
            m_pending_auxiliary_commands.camera2_light = commands.camera2_light;
        }
        if (commands.uv_on_off_changed)
        {
            m_pending_auxiliary_commands.uv_on_off_changed = true;
            m_pending_auxiliary_commands.uv_on_off = commands.uv_on_off;
        }
        if (commands.uv_power_changed)
        {
            m_pending_auxiliary_commands.uv_power_changed = true;
            m_pending_auxiliary_commands.uv_power = commands.uv_power;
        }
        if (commands.uv_time_changed)
        {
            m_pending_auxiliary_commands.uv_time_changed = true;
            m_pending_auxiliary_commands.uv_time = commands.uv_time;
        }
        m_pending_auxiliary_commands.force_sensor_bias =
            m_pending_auxiliary_commands.force_sensor_bias || commands.force_sensor_bias;
    }
    m_auxiliary_condition.notify_one();
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
        m_pm_client.set_timeout(m_config.opcua_timeout_ms);
        m_auxiliary_client.set_timeout(m_config.opcua_timeout_ms);
        m_pm_client.connect(m_config.opcua_endpoint);
        m_pm_client.init();
        m_auxiliary_client.connect(m_config.opcua_endpoint);
        m_auxiliary_client.init();
    }
    catch (std::exception &e)
    {
        m_auxiliary_client.disconnect();
        m_pm_client.disconnect();
        RCLCPP_ERROR(
            rclcpp::get_logger("PMSystem"),
            "Failed to connect or initialize OPC UA clients (%s).",
            e.what()
        );
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("PMSystem"),
        "Configured OPC UA clients (timeout=%u ms, auxiliary rates: IO=%.1f Hz, sensors=%.1f "
        "Hz, lighting/UV=%.1f Hz).",
        m_config.opcua_timeout_ms,
        m_config.auxiliary_io_poll_rate_hz,
        m_config.auxiliary_sensor_poll_rate_hz,
        m_config.auxiliary_lighting_poll_rate_hz
    );

    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_cleanup(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Cleaning up PMSystem...");

    stop_auxiliary_worker();
    m_auxiliary_client.disconnect();
    m_pm_client.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully cleaned up PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_shutdown(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Shutting down PMSystem...");

    stop_auxiliary_worker();
    m_auxiliary_client.disconnect();
    m_pm_client.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully shut down PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_activate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Activating PMSystem...");

    try
    {
        auto &robot = m_pm_client.get_robot();
        for (auto &axis : m_axes)
        {
            axis.initialize(robot);
        }

        PMClient::AuxiliaryIoState io_state{};
        PMClient::AuxiliarySensorState sensor_state{};
        PMClient::AuxiliaryLightingState lighting_state{};
        if (!m_auxiliary_client.read_auxiliary_io_state(io_state) ||
            !m_auxiliary_client.read_auxiliary_sensor_state(sensor_state) ||
            !m_auxiliary_client.read_auxiliary_lighting_state(lighting_state))
        {
            throw std::runtime_error("initial auxiliary OPC UA read failed");
        }

        {
            std::lock_guard<std::mutex> lock(m_auxiliary_mutex);
            m_auxiliary_io_state = io_state;
            m_auxiliary_sensor_state = sensor_state;
            m_auxiliary_lighting_state = lighting_state;
            m_pending_auxiliary_commands = PMClient::AuxiliaryCommands{};
        }
        apply_auxiliary_state();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMSystem"), "Failed to activate PMSystem: %s.", e.what()
        );
        return CallbackReturn::ERROR;
    }

    // Start command interfaces from the physical state so activation does not
    // generate light writes before a controller changes a command.
    m_camera1_coax_light_cmd = m_camera1_coax_light_state;
    for (std::size_t i = 0; i < 4; i++)
    {
        m_camera1_ring_light_cmd[i] = m_camera1_ring_light_state[i];
    }
    for (std::size_t i = 0; i < 3; i++)
    {
        m_camera1_ring_light_rgb_cmd[i] = m_camera1_ring_light_rgb_state[i];
    }
    m_camera2_light_cmd = m_camera2_light_state;

    start_auxiliary_worker();

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully activated PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_deactivate(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Deactivating PMSystem...");

    stop_auxiliary_worker();

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "Successfully deactivated PMSystem.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PMSystem::on_error(const State &previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("PMSystem"), "PMSystem encountered an error.");

    stop_auxiliary_worker();

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

    try
    {
        m_config.opcua_endpoint = hardware_info.hardware_parameters.at("opcua_endpoint");

        const auto read_positive_rate = [&](const char *name, double &destination) {
            const auto parameter = hardware_info.hardware_parameters.find(name);
            if (parameter == hardware_info.hardware_parameters.end())
            {
                return;
            }

            std::size_t parsed_characters = 0;
            const double value = std::stod(parameter->second, &parsed_characters);
            if (parsed_characters != parameter->second.size() || !std::isfinite(value) ||
                value <= 0.0)
            {
                throw std::invalid_argument(std::string(name) + " must be greater than zero");
            }
            destination = value;
        };

        const auto timeout_parameter = hardware_info.hardware_parameters.find("opcua_timeout_ms");
        if (timeout_parameter != hardware_info.hardware_parameters.end())
        {
            std::size_t parsed_characters = 0;
            const unsigned long timeout = std::stoul(timeout_parameter->second, &parsed_characters);
            if (parsed_characters != timeout_parameter->second.size() || timeout == 0 ||
                timeout > std::numeric_limits<unsigned int>::max())
            {
                throw std::invalid_argument("opcua_timeout_ms is invalid");
            }
            m_config.opcua_timeout_ms = static_cast<unsigned int>(timeout);
        }

        read_positive_rate("auxiliary_io_poll_rate_hz", m_config.auxiliary_io_poll_rate_hz);
        read_positive_rate(
            "auxiliary_sensor_poll_rate_hz", m_config.auxiliary_sensor_poll_rate_hz
        );
        read_positive_rate(
            "auxiliary_lighting_poll_rate_hz", m_config.auxiliary_lighting_poll_rate_hz
        );
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("PMSystem"), "Invalid PMSystem hardware parameter: %s.", e.what()
        );
        return CallbackReturn::ERROR;
    }

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
    //     "1K_Dispenser_Protection_Joint",
    //     "1K_Dispenser_Joint",
    //     "2K_Dispenser_Joint"
    // };

    std::vector<std::string> dummies{
        "Calibration_Qube_Joint",
        //"UV_Slider_X_Back_Joint",
        //"UV_Slider_X_Front_Joint",        // is now controlled via the moc hardware interface
    };

    for (auto &dummy : dummies)
    {
        state_interfaces.emplace_back(
            StateInterface(dummy, hardware_interface::HW_IF_POSITION, &m_dummy_state)
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

    state_interfaces.emplace_back(
        StateInterface("ReferenceCube", "Pushed", &reference_cube_pushed)
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

    auto start = std::chrono::high_resolution_clock::now();
    auto logger = rclcpp::get_logger("PMSystem");

    // The time-critical path is one OPC UA service call for all axis feedback.
    std::array<PMClient::AxisMotionState, 8> states{};
    static unsigned int consecutive_read_failures = 0;
    if (!m_pm_client.read_axis_motion_states(states))
    {
        ++consecutive_read_failures;
        if (consecutive_read_failures == 1 || consecutive_read_failures % 10 == 0)
        {
            RCLCPP_ERROR(
                logger,
                "Batched axis read failed (%u consecutive failures).",
                consecutive_read_failures
            );
        }
        return hardware_interface::return_type::ERROR;
    }
    consecutive_read_failures = 0;

    auto &robot = m_pm_client.get_robot();
    for (std::size_t i = 0; i < m_axes.size(); ++i)
    {
        m_axes[i].update(robot.get_axis(m_axes[i].id), states[i]);
    }
    apply_auxiliary_state();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    static unsigned int read_counter = 0;
    read_counter++;
    if (duration_ms.count() >= 20 || read_counter >= 100)
    {
        RCLCPP_WARN(
            logger,
            "PMSystem::read() took %lld ms total",
            static_cast<long long>(duration_ms.count())
        );
        read_counter = 0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PMSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    auto start = std::chrono::high_resolution_clock::now();
    auto logger = rclcpp::get_logger("PMSystem");

    auto &robot = m_pm_client.get_robot();
    // Build one sparse batch; unchanged targets and speeds are omitted.
    std::array<PMClient::AxisMotionCommand, 8> commands{};
    for (std::size_t i = 0; i < m_axes.size(); ++i)
    {
        commands[i] = m_axes[i].prepare_command(robot.get_axis(m_axes[i].id));
    }

    static unsigned int consecutive_write_failures = 0;
    const bool axis_write_succeeded = m_pm_client.write_axis_motion_commands(commands);
    if (axis_write_succeeded)
    {
        consecutive_write_failures = 0;
        for (std::size_t i = 0; i < m_axes.size(); ++i)
        {
            m_axes[i].mark_command_written(commands[i]);
        }
    }
    else
    {
        ++consecutive_write_failures;
        if (consecutive_write_failures == 1 || consecutive_write_failures % 10 == 0)
        {
            RCLCPP_ERROR(
                logger,
                "Batched axis write failed (%u consecutive failures).",
                consecutive_write_failures
            );
        }
    }

    // This only updates a mailbox and wakes the auxiliary worker. No auxiliary
    // OPC UA request runs on the ros2_control thread.
    queue_auxiliary_commands();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    static unsigned int write_counter = 0;
    write_counter++;
    if (duration_ms.count() >= 20 || write_counter >= 100)
    {
        RCLCPP_WARN(
            logger,
            "PMSystem::write() took %lld ms total",
            static_cast<long long>(duration_ms.count())
        );
        write_counter = 0;
    }

    return axis_write_succeeded ? hardware_interface::return_type::OK
                                : hardware_interface::return_type::ERROR;
}

//
// ---------------------------------------------
//
} // namespace pm_hardware_interface

PLUGINLIB_EXPORT_CLASS(pm_hardware_interface::PMSystem, hardware_interface::SystemInterface)
