#include "pluginlib/class_list_macros.hpp"

#include "pm_uv_controller/uv_controller.hpp"

namespace pm_uv_controller
{

PMUVController::PMUVController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMUVController"), "PMUVController instantiated.");
}

controller_interface::return_type PMUVController::init(
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

controller_interface::CallbackReturn PMUVController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_set_on_off_service = get_node()->create_service<UVSetOnOff>(
        "~/Hoenle_UV/SetOnOff",
        [this](
            const UVSetOnOff::Request::SharedPtr request,
            UVSetOnOff::Response::SharedPtr response
        ) {
            (void)response;
            std::copy_n(
                std::begin(request->turn_on),
                m_on_off_cmd.size(),
                std::begin(m_on_off_cmd)
            );
        }
    );

    m_get_on_off_service = get_node()->create_service<UVGetOnOff>(
        "~/Hoenle_UV/GetOnOff",
        [this](
            const UVGetOnOff::Request::SharedPtr request,
            UVGetOnOff::Response::SharedPtr response
        ) {
            (void)request;
            std::copy_n(
                std::begin(m_on_off_state),
                m_on_off_state.size(),
                std::begin(response->is_on)
            );
        }
    );

    m_set_power_service = get_node()->create_service<UVSetPower>(
        "~/Hoenle_UV/SetPower",
        [this](
            const UVSetPower::Request::SharedPtr request,
            UVSetPower::Response::SharedPtr response
        ) {
            (void)response;
            std::copy_n(std::begin(request->power), m_power_cmd.size(), std::begin(m_power_cmd));
        }
    );

    m_get_power_service = get_node()->create_service<UVGetPower>(
        "~/Hoenle_UV/GetPower",
        [this](
            const UVGetPower::Request::SharedPtr request,
            UVGetPower::Response::SharedPtr response
        ) {
            (void)request;
            std::copy_n(
                std::begin(m_power_state),
                m_power_state.size(),
                std::begin(response->power)
            );
        }
    );

    m_set_time_service = get_node()->create_service<UVSetTime>(
        "~/Hoenle_UV/SetTime",
        [this](
            const UVSetTime::Request::SharedPtr request,
            UVSetTime::Response::SharedPtr response
        ) {
            (void)response;
            std::copy_n(std::begin(request->time), m_time_cmd.size(), std::begin(m_time_cmd));
        }
    );

    m_get_time_service = get_node()->create_service<UVGetTime>(
        "~/Hoenle_UV/GetTime",
        [this](
            const UVGetTime::Request::SharedPtr request,
            UVGetTime::Response::SharedPtr response
        ) {
            (void)request;
            std::copy_n(std::begin(m_time_state), m_time_state.size(), std::begin(response->time));
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMUVController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PMUVController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/OnOff_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Power_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Time_" + std::to_string(i));
    }
    return config;
}

controller_interface::InterfaceConfiguration PMUVController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/OnOff_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Power_" + std::to_string(i));
    }
    for (auto i = 0; i < 4; i++)
    {
        config.names.emplace_back("HoenleUV/Time_" + std::to_string(i));
    }
    return config;
}

controller_interface::return_type
PMUVController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    for (auto i = 0; i < 4; i++)
    {
        if (m_on_off_cmd[i].has_value())
            command_interfaces_[i].set_value(static_cast<double>(*m_on_off_cmd[i]));
        m_on_off_state[i] = static_cast<bool>(state_interfaces_[i].get_value());

        if (m_power_cmd[i].has_value())
            command_interfaces_[4 + i].set_value(static_cast<double>(*m_power_cmd[i]));
        m_power_state[i] = static_cast<int>(state_interfaces_[4 + i].get_value());

        if (m_time_cmd[i].has_value())
            command_interfaces_[8 + i].set_value(static_cast<double>(*m_time_cmd[i]));
        m_time_state[i] = static_cast<double>(state_interfaces_[8 + i].get_value());

        m_on_off_cmd[i].reset();
        m_power_cmd[i].reset();
        m_time_cmd[i].reset();
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_uv_controller

PLUGINLIB_EXPORT_CLASS(pm_uv_controller::PMUVController, controller_interface::ControllerInterface)
