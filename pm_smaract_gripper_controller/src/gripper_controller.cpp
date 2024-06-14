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

    m_forces_pub = get_node()->create_publisher<GripperForces>("~/Forces/Stream", 1000);

    m_move_rel_srv = get_node()->create_service<GripperMoveRel>(
        "~/MoveRelative",
        [this](
            const GripperMoveRel::Request::SharedPtr request,
            GripperMoveRel::Response::SharedPtr response
        ) {
            if (request->offset < MIN_POSITION || request->offset > MAX_POSITION)
            {
                response->error_msg = "Position outside allowed range.";
                response->success = false;
                return;
            }

            m_target_position_rel = request->offset;
            response->error_msg = "No error.";
            response->success = true;
        }
    );

    m_move_abs_srv = get_node()->create_service<GripperMove>(
        "~/Move",
        [this](
            const GripperMove::Request::SharedPtr request,
            GripperMove::Response::SharedPtr response
        ) {
            if (request->target_position < MIN_POSITION || request->target_position > MAX_POSITION)
            {
                response->error_msg = "Position outside allowed range.";
                response->success = false;
                return;
            }

            m_target_position_abs = request->target_position;
            response->error_msg = "No error.";
            response->success = true;
        }
    );

    m_set_velocity_srv = get_node()->create_service<GripperSetVel>(
        "~/SetVelocity",
        [this](
            const GripperSetVel::Request::SharedPtr request,
            GripperSetVel::Response::SharedPtr response
        ) {
            m_target_velocity = request->target_velocity;
            response->success = true;
        }
    );

    m_set_acceleration_srv = get_node()->create_service<GripperSetAccel>(
        "~/SetAcceleration",
        [this](
            const GripperSetAccel::Request::SharedPtr request,
            GripperSetAccel::Response::SharedPtr response
        ) {
            m_target_acceleration = request->target_acceleration;
            response->success = true;
        }
    );

    m_get_velocity_srv = get_node()->create_service<GripperGetVel>(
        "~/GetVelocity",
        [this](
            const GripperGetVel::Request::SharedPtr request,
            GripperGetVel::Response::SharedPtr response
        ) {
            (void)request;
            response->current_velocity = m_current_velocity;
        }
    );

    m_get_acceleration_srv = get_node()->create_service<GripperGetAccel>(
        "~/GetAcceleration",
        [this](
            const GripperGetAccel::Request::SharedPtr request,
            GripperGetAccel::Response::SharedPtr response
        ) {
            (void)request;
            response->current_acceleration = m_current_acceleration;
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
    config.names = {
        "Gripper/TargetPositionRel",
        "Gripper/TargetPositionAbs",
        "Gripper/TargetVelocity",
        "Gripper/TargetAcceleration",
    };
    return config;
}

controller_interface::InterfaceConfiguration
PMGripperController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Gripper/Position",
        "Gripper/ForceX",
        "Gripper/ForceY",
        "Gripper/ForceZ",
        "Gripper/Velocity",
        "Gripper/Acceleration",
    };
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

    {
        GripperForces msg;
        msg.fx = state_interfaces_[1].get_value();
        msg.fy = state_interfaces_[2].get_value();
        msg.fz = state_interfaces_[3].get_value();
        m_forces_pub->publish(msg);
    }

    if (!std::isnan(m_target_position_rel))
    {
        command_interfaces_[0].set_value(m_target_position_rel);
        m_target_position_rel = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_target_position_abs))
    {
        command_interfaces_[1].set_value(m_target_position_abs);
        m_target_position_abs = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_target_velocity))
    {
        command_interfaces_[2].set_value(m_target_velocity);
        m_target_velocity = double_limits::quiet_NaN();
    }

    if (!std::isnan(m_target_acceleration))
    {
        command_interfaces_[3].set_value(m_target_acceleration);
        m_target_acceleration = double_limits::quiet_NaN();
    }

    m_current_velocity = state_interfaces_[4].get_value();
    m_current_acceleration = state_interfaces_[5].get_value();

    return controller_interface::return_type::OK;
}

} // namespace pm_smaract_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    pm_smaract_gripper_controller::PMGripperController, controller_interface::ControllerInterface
)
