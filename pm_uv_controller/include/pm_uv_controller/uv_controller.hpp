#pragma once

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/srv/uv_get_on_off.hpp"
#include "pm_msgs/srv/uv_get_power.hpp"
#include "pm_msgs/srv/uv_get_time.hpp"
#include "pm_msgs/srv/uv_set_on_off.hpp"
#include "pm_msgs/srv/uv_set_power.hpp"
#include "pm_msgs/srv/uv_set_time.hpp"

namespace pm_uv_controller
{

using namespace pm_msgs::srv;

class PMUVController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Service<UVSetOnOff>::SharedPtr m_set_on_off_service;
    std::array<bool, 4> m_on_off_cmd;

    rclcpp::Service<UVGetOnOff>::SharedPtr m_get_on_off_service;
    std::array<bool, 4> m_on_off_state;

    rclcpp::Service<UVSetPower>::SharedPtr m_set_power_service;
    std::array<int, 4> m_power_cmd;

    rclcpp::Service<UVGetPower>::SharedPtr m_get_power_service;
    std::array<int, 4> m_power_state;

    rclcpp::Service<UVSetTime>::SharedPtr m_set_time_service;
    std::array<double, 4> m_time_cmd;

    rclcpp::Service<UVGetTime>::SharedPtr m_get_time_service;
    std::array<double, 4> m_time_state;

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
