#pragma once

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/msg/camera1_coax_light_cmd.hpp"
#include "pm_msgs/msg/camera1_ring_light_cmd.hpp"
#include "pm_msgs/msg/camera2_light_cmd.hpp"

namespace pm_gpio_controller
{

using namespace pm_msgs::msg;

class PMGpioController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Subscription<Camera1CoaxLightCmd>::SharedPtr m_camera1_coax_light_sub;
    bool m_camera1_coax_light_cmd;

    rclcpp::Subscription<Camera1RingLightCmd>::SharedPtr m_camera1_ring_light_sub;
    bool m_camera1_ring_light_cmd[4];
    int m_camera1_ring_light_color_cmd[3];

    rclcpp::Subscription<Camera2LightCmd>::SharedPtr m_camera2_light_sub;
    bool m_camera2_light_cmd;

  public:
    PMGpioController();

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

} // namespace pm_gpio_controller
