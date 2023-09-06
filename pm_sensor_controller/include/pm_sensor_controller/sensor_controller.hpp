#pragma once

#include "controller_interface/controller_interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace pm_sensor_controller
{

class PMSensorController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_laser_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_force_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_force_bias_sub;
    bool m_force_sensor_bias_cmd = false;

  public:
    PMSensorController();

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

} // namespace pm_sensor_controller
