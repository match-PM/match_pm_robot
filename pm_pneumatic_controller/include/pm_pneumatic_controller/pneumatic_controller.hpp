#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/srv/empty_with_success.hpp"
#include "pm_msgs/srv/pneumatic_get_position.hpp"
#include "pm_msgs/srv/pneumatic_set_position.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pm_pneumatic_controller_parameters.hpp"

namespace pm_pneumatic_controller
{

using namespace pm_msgs::srv;

constexpr int POSITION_FORWARD = 1;
constexpr int POSITION_BACK = -1;

class PMPneumaticController : public controller_interface::ControllerInterface
{
  private:
    std::shared_ptr<ParamListener> m_param_listener;
    Params m_params;

    bool m_initialized = false;

    std::vector<rclcpp::Service<EmptyWithSuccess>::SharedPtr> m_forward_services;
    std::vector<rclcpp::Service<EmptyWithSuccess>::SharedPtr> m_backward_services;
    std::vector<rclcpp::Service<PneumaticSetPosition>::SharedPtr> m_set_position;
    std::vector<int> m_commands;

    std::vector<rclcpp::Service<PneumaticGetPosition>::SharedPtr> m_get_position;
    std::vector<int> m_positions;

    std::vector<double> m_lower_limits;
    std::vector<double> m_upper_limits;

    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> m_position_publishers;

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

    void wait_until_in_position(int cylinder, int target_position);
};

} // namespace pm_pneumatic_controller
