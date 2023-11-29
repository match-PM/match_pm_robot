#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "pm_msgs/srv/nozzle_get_position.hpp"
#include "pm_msgs/srv/nozzle_set_position.hpp"

#include "pm_nozzle_controller_parameters.hpp"

namespace pm_nozzle_controller
{

using namespace pm_msgs::srv;

class PMNozzleController : public controller_interface::ControllerInterface
{
  private:
    std::shared_ptr<ParamListener> m_param_listener;
    Params m_params;

    std::vector<rclcpp::Service<NozzleSetPosition>::SharedPtr> m_set_position;
    std::vector<int> m_commands;

    std::vector<rclcpp::Service<NozzleGetPosition>::SharedPtr> m_get_position;
    std::vector<int> m_positions;

  public:
    PMNozzleController();

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

} // namespace pm_nozzle_controller
