#pragma once

#include <array>

#include "controller_interface/controller_interface.hpp"

#include "std_msgs/msg/float64.hpp"

#include "pm_msgs/srv/gripper_move.hpp"

namespace pm_smaract_gripper_controller
{

using namespace pm_msgs::srv;

class PMGripperController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_current_position_pub;
    rclcpp::Service<GripperMove>::SharedPtr m_move_srv;

    bool m_got_move_command{false};
    double m_target_position{0.0};

  public:
    PMGripperController();

    controller_interface::return_type init(
        const std::string &controller_name, const std::string &namespace_ = "",
        const rclcpp::NodeOptions &node_options =
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)
    ) override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state
    ) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state
    ) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state
    ) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type
    update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
};

} // namespace pm_smaract_gripper_controller
