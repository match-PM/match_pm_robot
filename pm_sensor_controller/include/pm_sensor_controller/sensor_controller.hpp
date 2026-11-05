#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "pm_client/client.hpp"
#include "pm_msgs/msg/laser_measurement.hpp"

#include "controller_interface/controller_interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "pm_msgs/srv/force_sensor_bias.hpp"
#include "pm_msgs/srv/force_sensor_get_measurement.hpp"
#include "pm_msgs/srv/laser_get_measurement.hpp"
#include "pm_msgs/srv/reference_cube_state.hpp"

namespace pm_sensor_controller
{

using namespace pm_msgs::srv;

class PMSensorController : public controller_interface::ControllerInterface
{
  private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_reference_cube_pub;
    rclcpp::Publisher<pm_msgs::msg::LaserMeasurement>::SharedPtr m_laser_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_force_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_force_bias_sub;

    rclcpp::Service<ForceSensorGetMeasurement>::SharedPtr m_force_sensor_get_measurement;
    std::array<double, 6> m_force_sensor_measurement{};

    rclcpp::Service<ForceSensorBias>::SharedPtr m_force_sensor_bias;
    bool m_force_sensor_bias_cmd = false;

    rclcpp::Service<LaserGetMeasurement>::SharedPtr m_laser_get_measurement;
    using Clock = std::chrono::steady_clock;
    std::thread m_laser_thread;
    std::atomic<bool> m_laser_running{false};
    std::mutex m_laser_mutex;
    std::condition_variable m_laser_condition;
    PMClient::LaserMeasurementState m_laser_sample;
    Clock::time_point m_laser_received{};
    std::string m_laser_endpoint;
    unsigned int m_laser_timeout_ms = 120000;
    unsigned int m_laser_rpc_timeout_ms = 500;
    std::uint64_t m_laser_token = 0;
    bool m_laser_pending = false;
    bool m_laser_ready = false;
    Clock::time_point m_laser_deadline{};
    std::shared_ptr<rmw_request_id_t> m_laser_request_header;
    rclcpp::Service<LaserGetMeasurement>::SharedPtr m_laser_response_service;
    LaserGetMeasurement::Response m_laser_response;
    PMClient::LaserMeasurementState m_laser_completion_sample;
    Clock::time_point m_laser_completion_received{};
    rclcpp::TimerBase::SharedPtr m_laser_response_timer;

    void laser_worker();
    void stop_laser_worker();
    void complete_laser_response();

    rclcpp::Service<ReferenceCubeState>::SharedPtr m_reference_cube_state;
    bool m_reference_cube_pressed = false;

  public:
    PMSensorController();
    ~PMSensorController() override;

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
