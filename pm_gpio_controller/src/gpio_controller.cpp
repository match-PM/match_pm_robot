#include "pluginlib/class_list_macros.hpp"

#include "pm_gpio_controller/gpio_controller.hpp"

namespace pm_gpio_controller
{

PMGpioController::PMGpioController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMGpioController"), "PMGpioController instantiated.");
}

controller_interface::return_type PMGpioController::init(
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

controller_interface::CallbackReturn PMGpioController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGpioController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    m_camera1_coax_light_sub = get_node()->create_subscription<Camera1CoaxLightCmd>(
        "~/camera1_coax_light",
        rclcpp::SystemDefaultsQoS(),
        [this](const Camera1CoaxLightCmd::SharedPtr msg) { m_camera1_coax_light_cmd = msg->on; }
    );

    m_camera1_ring_light_sub = get_node()->create_subscription<Camera1RingLightCmd>(
        "~/camera1_ring_light",
        rclcpp::SystemDefaultsQoS(),
        [this](const Camera1RingLightCmd::SharedPtr msg) {
            for (auto i = 0; i < 4; i++)
            {
                m_camera1_ring_light_cmd[i] = msg->on[i];
            }
            for (auto i = 0; i < 3; i++)
            {
                m_camera1_ring_light_color_cmd[i] = msg->rgb[i];
            }
        }
    );

    m_camera2_light_sub = get_node()->create_subscription<Camera2LightCmd>(
        "~/camera2_light",
        rclcpp::SystemDefaultsQoS(),
        [this](const Camera2LightCmd::SharedPtr msg) { m_camera2_light_cmd = msg->intensity; }
    );
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGpioController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    m_camera1_coax_light_cmd = static_cast<bool>(state_interfaces_[0].get_value());

    m_camera1_ring_light_cmd[0] = static_cast<bool>(state_interfaces_[1].get_value());
    m_camera1_ring_light_cmd[1] = static_cast<bool>(state_interfaces_[2].get_value());
    m_camera1_ring_light_cmd[2] = static_cast<bool>(state_interfaces_[3].get_value());
    m_camera1_ring_light_cmd[3] = static_cast<bool>(state_interfaces_[4].get_value());

    m_camera1_ring_light_color_cmd[0] = static_cast<int>(state_interfaces_[5].get_value());
    m_camera1_ring_light_color_cmd[1] = static_cast<int>(state_interfaces_[6].get_value());
    m_camera1_ring_light_color_cmd[2] = static_cast<int>(state_interfaces_[7].get_value());

    m_camera2_light_cmd = static_cast<int>(state_interfaces_[8].get_value());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMGpioController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMGpioController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Camera1_Coax_Light/On_Off",
        "Camera1_Ring_Light/1_On_Off",
        "Camera1_Ring_Light/2_On_Off",
        "Camera1_Ring_Light/3_On_Off",
        "Camera1_Ring_Light/4_On_Off",
        "Camera1_Ring_Light/Red",
        "Camera1_Ring_Light/Green",
        "Camera1_Ring_Light/Blue",
        "Camera2_Light/Intensity"};
    return config;
}

controller_interface::InterfaceConfiguration PMGpioController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Camera1_Coax_Light/On_Off",
        "Camera1_Ring_Light/1_On_Off",
        "Camera1_Ring_Light/2_On_Off",
        "Camera1_Ring_Light/3_On_Off",
        "Camera1_Ring_Light/4_On_Off",
        "Camera1_Ring_Light/Red",
        "Camera1_Ring_Light/Green",
        "Camera1_Ring_Light/Blue",
        "Camera2_Light/Intensity"};
    return config;
}

controller_interface::return_type
PMGpioController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    command_interfaces_[0].set_value(static_cast<double>(m_camera1_coax_light_cmd));

    command_interfaces_[1].set_value(static_cast<double>(m_camera1_ring_light_cmd[0]));
    command_interfaces_[2].set_value(static_cast<double>(m_camera1_ring_light_cmd[1]));
    command_interfaces_[3].set_value(static_cast<double>(m_camera1_ring_light_cmd[2]));
    command_interfaces_[4].set_value(static_cast<double>(m_camera1_ring_light_cmd[3]));

    command_interfaces_[5].set_value(static_cast<double>(m_camera1_ring_light_color_cmd[0]));
    command_interfaces_[6].set_value(static_cast<double>(m_camera1_ring_light_color_cmd[1]));
    command_interfaces_[7].set_value(static_cast<double>(m_camera1_ring_light_color_cmd[2]));

    command_interfaces_[8].set_value(static_cast<double>(m_camera2_light_cmd));

    return controller_interface::return_type::OK;
}

} // namespace pm_gpio_controller

PLUGINLIB_EXPORT_CLASS(
    pm_gpio_controller::PMGpioController, controller_interface::ControllerInterface
)
