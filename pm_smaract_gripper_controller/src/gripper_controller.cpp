#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

#include "pm_smaract_gripper_controller/gripper_controller.hpp"

namespace pm_smaract_gripper_controller
{

PMGripperController::PMGripperController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMGripperController"), "PMGripperController instantiated.");
}

controller_interface::return_type PMGripperController::init(
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

controller_interface::CallbackReturn PMGripperController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGripperController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_current_position_pub =
        get_node()->create_publisher<std_msgs::msg::Float64>("~/Position/Stream", 1000);

    m_move_srv = get_node()->create_service<GripperMove>(
        "~/Move",
        [this](
            const GripperMove::Request::SharedPtr request,
            GripperMove::Response::SharedPtr response
        ) {
            m_target_position = request->target_position;
            m_got_move_command = true;
            response->success = true;
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGripperController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGripperController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMGripperController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {"Gripper/TargetPosition"};
    return config;
}

controller_interface::InterfaceConfiguration
PMGripperController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {"Gripper/Position"};
    return config;
}

controller_interface::return_type
PMGripperController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    {
        std_msgs::msg::Float64 msg;
        msg.data = state_interfaces_[0].get_value();
        m_current_position_pub->publish(msg);
    }

    if (m_got_move_command)
    {
        m_got_move_command = false;
        command_interfaces_[0].set_value(m_target_position);
    }

    return controller_interface::return_type::OK;
}

} // namespace pm_smaract_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    pm_smaract_gripper_controller::PMGripperController, controller_interface::ControllerInterface
)
