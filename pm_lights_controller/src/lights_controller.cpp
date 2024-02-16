#include "pluginlib/class_list_macros.hpp"

#include "pm_lights_controller/lights_controller.hpp"

namespace pm_lights_controller
{

PMLightsController::PMLightsController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMLightsController"), "PMLightsController instantiated.");
}

controller_interface::return_type PMLightsController::init(
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

controller_interface::CallbackReturn PMLightsController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLightsController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    m_coax_light_set_service = get_node()->create_service<CoaxLightSetState>(
        "~/CoaxLight/SetState",
        [this](
            const CoaxLightSetState::Request::SharedPtr request,
            CoaxLightSetState::Response::SharedPtr response
        ) {
            (void)response;
            m_coax_light_command = request->turn_on;
        }
    );

    m_coax_light_get_service = get_node()->create_service<CoaxLightGetState>(
        "~/CoaxLight/GetState",
        [this](
            const CoaxLightGetState::Request::SharedPtr request,
            CoaxLightGetState::Response::SharedPtr response
        ) {
            (void)request;
            response->is_on = m_coax_light_state;
        }
    );

    m_cam2_light_set_service = get_node()->create_service<Cam2LightSetState>(
        "~/Cam2Light/SetState",
        [this](
            const Cam2LightSetState::Request::SharedPtr request,
            Cam2LightSetState::Response::SharedPtr response
        ) {
            (void)response;
            m_cam2_light_command = request->intensity;
        }
    );

    m_cam2_light_get_service = get_node()->create_service<Cam2LightGetState>(
        "~/Cam2Light/GetState",
        [this](
            const Cam2LightGetState::Request::SharedPtr request,
            Cam2LightGetState::Response::SharedPtr response
        ) {
            (void)request;
            response->intensity = m_cam2_light_state;
        }
    );

    m_ring_light_set_service = get_node()->create_service<RingLightSetState>(
        "~/RingLight/SetState",
        [this](
            const RingLightSetState::Request::SharedPtr request,
            RingLightSetState::Response::SharedPtr response
        ) {
            (void)response;

            RCLCPP_INFO(
                get_node()->get_logger(),
                "RGB CALL: %i %i %i\n",
                (int)request->rgb[0],
                (int)request->rgb[1],
                (int)request->rgb[2]
            );

            std::copy_n(std::begin(request->turn_on), 4, std::begin(m_ring_light_command));
            std::copy_n(std::begin(request->rgb), 3, std::begin(m_ring_light_rgb_command));
        }
    );

    m_ring_light_get_service = get_node()->create_service<RingLightGetState>(
        "~/RingLight/GetState",
        [this](
            const RingLightGetState::Request::SharedPtr request,
            RingLightGetState::Response::SharedPtr response
        ) {
            (void)request;

            std::copy_n(std::begin(m_ring_light_state), 4, std::begin(response->is_on));
            std::copy_n(std::begin(m_ring_light_rgb_state), 3, std::begin(response->rgb));
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLightsController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    m_coax_light_command = m_coax_light_state = static_cast<bool>(state_interfaces_[0].get_value());
    m_ring_light_command[0] = m_ring_light_state[0] =
        static_cast<bool>(state_interfaces_[1].get_value());
    m_ring_light_command[1] = m_ring_light_state[1] =
        static_cast<bool>(state_interfaces_[2].get_value());
    m_ring_light_command[2] = m_ring_light_state[2] =
        static_cast<bool>(state_interfaces_[3].get_value());
    m_ring_light_command[3] = m_ring_light_state[3] =
        static_cast<bool>(state_interfaces_[4].get_value());
    m_ring_light_rgb_command[0] = m_ring_light_rgb_state[0] =
        static_cast<int>(state_interfaces_[5].get_value());
    m_ring_light_rgb_command[1] = m_ring_light_rgb_state[1] =
        static_cast<int>(state_interfaces_[6].get_value());
    m_ring_light_rgb_command[2] = m_ring_light_rgb_state[2] =
        static_cast<int>(state_interfaces_[7].get_value());
    m_cam2_light_command = m_cam2_light_state = static_cast<int>(state_interfaces_[8].get_value());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMLightsController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMLightsController::command_interface_configuration() const
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
        "Camera2_Light/Intensity"
    };
    return config;
}

controller_interface::InterfaceConfiguration
PMLightsController::state_interface_configuration() const
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
        "Camera2_Light/Intensity"
    };
    return config;
}

controller_interface::return_type
PMLightsController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    m_coax_light_state = static_cast<bool>(state_interfaces_[0].get_value());
    m_ring_light_state[0] = static_cast<bool>(state_interfaces_[1].get_value());
    m_ring_light_state[1] = static_cast<bool>(state_interfaces_[2].get_value());
    m_ring_light_state[2] = static_cast<bool>(state_interfaces_[3].get_value());
    m_ring_light_state[3] = static_cast<bool>(state_interfaces_[4].get_value());
    m_ring_light_rgb_state[0] = static_cast<int>(state_interfaces_[5].get_value());
    m_ring_light_rgb_state[1] = static_cast<int>(state_interfaces_[6].get_value());
    m_ring_light_rgb_state[2] = static_cast<int>(state_interfaces_[7].get_value());
    m_cam2_light_state = static_cast<int>(state_interfaces_[8].get_value());

    command_interfaces_[0].set_value(static_cast<double>(m_coax_light_command));
    command_interfaces_[1].set_value(static_cast<double>(m_ring_light_command[0]));
    command_interfaces_[2].set_value(static_cast<double>(m_ring_light_command[1]));
    command_interfaces_[3].set_value(static_cast<double>(m_ring_light_command[2]));
    command_interfaces_[4].set_value(static_cast<double>(m_ring_light_command[3]));
    command_interfaces_[5].set_value(static_cast<double>(m_ring_light_rgb_command[0]));
    command_interfaces_[6].set_value(static_cast<double>(m_ring_light_rgb_command[1]));
    command_interfaces_[7].set_value(static_cast<double>(m_ring_light_rgb_command[2]));
    command_interfaces_[8].set_value(static_cast<double>(m_cam2_light_command));

    return controller_interface::return_type::OK;
}

} // namespace pm_lights_controller

PLUGINLIB_EXPORT_CLASS(
    pm_lights_controller::PMLightsController, controller_interface::ControllerInterface
)
