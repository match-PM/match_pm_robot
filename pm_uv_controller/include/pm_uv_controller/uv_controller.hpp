#pragma once

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/msg/uv_on_off.hpp"
#include "pm_msgs/msg/uv_power_cmd.hpp"
#include "pm_msgs/msg/uv_time_cmd.hpp"

namespace pm_uv_controller
{

using namespace pm_msgs::msg;

class PMUVController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Publisher<UVOnOff>::SharedPtr m_on_off_pub;

    rclcpp::Subscription<UVOnOff>::SharedPtr m_on_off_sub;
    std::array<bool, 4> m_on_off_cmd{};

    rclcpp::Subscription<UVPowerCmd>::SharedPtr m_power_sub;
    std::array<int, 4> m_power_cmd{};

    rclcpp::Subscription<UVTimeCmd>::SharedPtr m_time_sub;
    std::array<double, 4> m_time_cmd{};

  public:
    PMUVController();

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

} // namespace pm_uv_controller
