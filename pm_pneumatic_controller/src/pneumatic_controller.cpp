#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"

#include "urdf/model.h"

#include "pm_pneumatic_controller/pneumatic_controller.hpp"

namespace pm_pneumatic_controller
{

using namespace std::chrono_literals;

PMPneumaticController::PMPneumaticController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMPneumaticController"), "PMPneumaticController instantiated.");
}

controller_interface::return_type PMPneumaticController::init(
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

controller_interface::CallbackReturn PMPneumaticController::on_init()
{
    m_param_listener = std::make_shared<ParamListener>(get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_params = m_param_listener->get_params();

    if (m_params.cylinders.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'cylinders' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    m_commands.resize(m_params.cylinders.size(), -1);
    m_positions.resize(m_params.cylinders.size(), -1);
    m_upper_limits.resize(m_params.cylinders.size(), 0.0);
    m_lower_limits.resize(m_params.cylinders.size(), 0.0);

    auto get_parameters_srv = get_node()->create_client<rcl_interfaces::srv::GetParameters>(
        "robot_state_publisher/get_parameters"
    );
    int retries = 5;
    while (!get_parameters_srv->wait_for_service(2s))
    {
        --retries;
        if (retries <= 0)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Service robot_state_publisher/get_parameters is not available"
            );
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_WARN(
            get_node()->get_logger(),
            "Waiting for robot_state_publisher/get_parameters..."
        );
    }
    auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    req->names.emplace_back("robot_description");
    auto res = get_parameters_srv->async_send_request(req);

    urdf::Model urdf_model;
    urdf_model.initString(res.get()->values[0].string_value.c_str());

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        const auto &cylinder_name = m_params.cylinders[i];
        RCLCPP_INFO(get_node()->get_logger(), "\t%s", cylinder_name.c_str());
        const auto *cylinder_limits = urdf_model.getJoint(cylinder_name)->limits.get();

        m_lower_limits[i] = cylinder_limits->lower;
        m_upper_limits[i] = cylinder_limits->upper;

        std::string topic_name = cylinder_name;
        if (isdigit(cylinder_name[0]))
        {
            // Topic names arent allowed to start with digits so just prepend
            // some letter to make it work.
            topic_name = std::string("N") + topic_name;
        }

        auto forward_service = get_node()->create_service<EmptyWithSuccess>(
            "~/" + topic_name + "/MoveForward",
            [this,
             i](const EmptyWithSuccess::Request::SharedPtr request,
                EmptyWithSuccess::Response::SharedPtr response) {
                (void)request;
                m_commands[i] = POSITION_FORWARD;

                wait_until_in_position(i, POSITION_FORWARD);

                response->success = true;
            }
        );
        m_forward_services.emplace_back(forward_service);

        auto backward_service = get_node()->create_service<EmptyWithSuccess>(
            "~/" + topic_name + "/MoveBackward",
            [this,
             i](const EmptyWithSuccess::Request::SharedPtr request,
                EmptyWithSuccess::Response::SharedPtr response) {
                (void)request;
                m_commands[i] = POSITION_BACK;

                wait_until_in_position(i, POSITION_BACK);

                response->success = true;
            }
        );
        m_backward_services.emplace_back(backward_service);

        auto set_service = get_node()->create_service<PneumaticSetPosition>(
            "~/" + topic_name + "/SetPosition",
            [this,
             i](const PneumaticSetPosition::Request::SharedPtr request,
                PneumaticSetPosition::Response::SharedPtr response) {
                (void)response;
                m_commands[i] = request->command;
            }
        );
        m_set_position.emplace_back(set_service);

        auto get_service = get_node()->create_service<PneumaticGetPosition>(
            "~/" + topic_name + "/GetPosition",
            [this,
             i](const PneumaticGetPosition::Request::SharedPtr request,
                PneumaticGetPosition::Response::SharedPtr response) {
                (void)request;
                response->position = m_positions[i];
            }
        );
        m_get_position.emplace_back(get_service);

        auto publisher =
            get_node()->create_publisher<std_msgs::msg::Bool>("~/" + topic_name + "/IsForward", 10);
        m_position_publishers.emplace_back(publisher);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

void PMPneumaticController::wait_until_in_position(int cylinder, int target_position)
{
    while (m_positions[cylinder] != target_position)
    {
        std::this_thread::sleep_for(50ms);
    }
}

controller_interface::CallbackReturn
PMPneumaticController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMPneumaticController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.cylinders)
    {
        config.names.emplace_back(interface + "/Move_Command");
        config.names.emplace_back(interface + "/UpperLimit");
        config.names.emplace_back(interface + "/LowerLimit");
    }

    return config;
}

controller_interface::InterfaceConfiguration
PMPneumaticController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &interface : m_params.cylinders)
    {
        config.names.emplace_back(interface + "/FwdOrBwd");
    }

    return config;
}

controller_interface::return_type
PMPneumaticController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        m_positions[i] = static_cast<int>(state_interfaces_[i].get_value());

        std_msgs::msg::Bool msg;
        msg.data = m_positions[i] == POSITION_FORWARD;
        m_position_publishers[i]->publish(msg);
    }

    if (!m_initialized)
    {
        std::copy(std::begin(m_positions), std::end(m_positions), std::begin(m_commands));
        m_initialized = true;
    }

    for (std::size_t i = 0; i < m_params.cylinders.size(); i++)
    {
        command_interfaces_[3 * i + 0].set_value(static_cast<double>(m_commands[i]));
        command_interfaces_[3 * i + 1].set_value(static_cast<double>(m_upper_limits[i]));
        command_interfaces_[3 * i + 2].set_value(static_cast<double>(m_lower_limits[i]));
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_pneumatic_controller

PLUGINLIB_EXPORT_CLASS(
    pm_pneumatic_controller::PMPneumaticController, controller_interface::ControllerInterface
)
