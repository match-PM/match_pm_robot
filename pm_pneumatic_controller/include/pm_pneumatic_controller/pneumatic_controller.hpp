#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/msg/pneumatic_cylinder_cmd.hpp"
namespace pm_pneumatic_controller
{

using namespace pm_msgs::msg;

class PMPneumaticController : public controller_interface::ControllerInterface
{
  private:
    std::vector<std::string> m_cylinder_names{
        "UV1_Pneumatic",
        "UV2_Pneumatic",
        "Glue_Pneumatic",
        "Glue_2K_Pneumatic",
        "Camera_Mire_Pneumatic",
    };
    std::vector<rclcpp::Subscription<PneumaticCylinderCmd>::SharedPtr> m_subscriptions;
    std::vector<bool> m_cylinder_cmds;

  public:
    PMPneumaticController();

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

} // namespace pm_pneumatic_controller
