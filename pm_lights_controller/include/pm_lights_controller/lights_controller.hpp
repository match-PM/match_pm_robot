#pragma once

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/srv/cam2_light_get_state.hpp"
#include "pm_msgs/srv/cam2_light_set_state.hpp"
#include "pm_msgs/srv/coax_light_get_state.hpp"
#include "pm_msgs/srv/coax_light_set_state.hpp"
#include "pm_msgs/srv/ring_light_get_state.hpp"
#include "pm_msgs/srv/ring_light_set_state.hpp"

namespace pm_lights_controller
{

using namespace pm_msgs::srv;

class PMLightsController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Service<CoaxLightSetState>::SharedPtr m_coax_light_set_service;
    bool m_coax_light_command;

    rclcpp::Service<CoaxLightGetState>::SharedPtr m_coax_light_get_service;
    bool m_coax_light_state;

    rclcpp::Service<Cam2LightSetState>::SharedPtr m_cam2_light_set_service;
    int m_cam2_light_command;

    rclcpp::Service<Cam2LightGetState>::SharedPtr m_cam2_light_get_service;
    int m_cam2_light_state;

    rclcpp::Service<RingLightSetState>::SharedPtr m_ring_light_set_service;
    std::array<bool, 4> m_ring_light_command;
    std::array<int, 8> m_ring_light_rgb_command;

    rclcpp::Service<RingLightGetState>::SharedPtr m_ring_light_get_service;
    std::array<bool, 4> m_ring_light_state;
    std::array<int, 8> m_ring_light_rgb_state;

  public:
    PMLightsController();

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

} // namespace pm_lights_controller
