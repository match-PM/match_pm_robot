#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

#include "pm_nozzle_controller/nozzle_controller.hpp"

namespace pm_nozzle_controller
{

using namespace std::chrono_literals;

PMNozzleController::PMNozzleController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMNozzleController"), "PMNozzleController instantiated.");
}

controller_interface::return_type PMNozzleController::init(
    const std::string &controller_name, const std::string &namespace_,
    const rclcpp::NodeOptions &node_options
)
{
    return controller_interface::ControllerInterface::init(
        controller_name,
        namespace_,
        node_options
    );
}

controller_interface::CallbackReturn PMNozzleController::on_init()
{
    m_param_listener = std::make_shared<ParamListener>(get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMNozzleController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_params = m_param_listener->get_params();

    if (m_params.nozzles.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'nozzles' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    m_commands.resize(m_params.nozzles.size(), 0);
    m_positions.resize(m_params.nozzles.size(), 0);

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        auto vacuum_service = get_node()->create_service<EmptyWithSuccess>(
            "~/" + m_params.nozzles[i] + "/Vacuum",
            [this,
             i](const EmptyWithSuccess::Request::SharedPtr request,
                EmptyWithSuccess::Response::SharedPtr response) {
                (void)request;
                m_commands[i] = STATE_VACUUM;

                wait_for_state(i, STATE_VACUUM);

                response->success = true;
            }
        );
        m_vacuum_services.emplace_back(vacuum_service);

        auto pressure_service = get_node()->create_service<EmptyWithSuccess>(
            "~/" + m_params.nozzles[i] + "/Pressure",
            [this,
             i](const EmptyWithSuccess::Request::SharedPtr request,
                EmptyWithSuccess::Response::SharedPtr response) {
                (void)request;
                m_commands[i] = STATE_PRESSURE;

                wait_for_state(i, STATE_PRESSURE);

                response->success = true;
            }
        );
        m_pressure_services.emplace_back(pressure_service);

        auto off_service = get_node()->create_service<EmptyWithSuccess>(
            "~/" + m_params.nozzles[i] + "/TurnOff",
            [this,
             i](const EmptyWithSuccess::Request::SharedPtr request,
                EmptyWithSuccess::Response::SharedPtr response) {
                (void)request;
                m_commands[i] = STATE_IDLE;

                wait_for_state(i, STATE_IDLE);

                response->success = true;
            }
        );
        m_off_services.emplace_back(off_service);

        auto set_service = get_node()->create_service<NozzleSetPosition>(
            "~/" + m_params.nozzles[i] + "/SetPosition",
            [this,
             i](const NozzleSetPosition::Request::SharedPtr request,
                NozzleSetPosition::Response::SharedPtr response) {
                (void)response;
                m_commands[i] = request->command;
            }
        );
        m_set_position.emplace_back(set_service);

        auto get_service = get_node()->create_service<NozzleGetPosition>(
            "~/" + m_params.nozzles[i] + "/GetPosition",
            [this,
             i](const NozzleGetPosition::Request::SharedPtr request,
                NozzleGetPosition::Response::SharedPtr response) {
                (void)request;
                response->position = m_positions[i];
            }
        );
        m_get_position.emplace_back(get_service);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

void PMNozzleController::wait_for_state(int nozzle, int target_state)
{
    while (m_positions[nozzle] != target_state)
    {
        std::this_thread::sleep_for(50ms);
    }
}

controller_interface::CallbackReturn
PMNozzleController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMNozzleController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMNozzleController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.nozzles)
    {
        config.names.emplace_back(interface + "/State");
    }

    return config;
}

controller_interface::InterfaceConfiguration
PMNozzleController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.nozzles)
    {
        config.names.emplace_back(interface + "/State");
    }

    return config;
}

controller_interface::return_type
PMNozzleController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        command_interfaces_[i].set_value(static_cast<double>(m_commands[i]));
    }

    for (std::size_t i = 0; i < m_params.nozzles.size(); i++)
    {
        m_positions[i] = static_cast<int>(state_interfaces_[i].get_value());
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_nozzle_controller

PLUGINLIB_EXPORT_CLASS(
    pm_nozzle_controller::PMNozzleController, controller_interface::ControllerInterface
)
