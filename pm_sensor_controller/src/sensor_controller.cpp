#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

#include "pm_sensor_controller/sensor_controller.hpp"

namespace pm_sensor_controller
{
namespace
{
pm_msgs::msg::LaserMeasurement ToMessage(const PMClient::LaserMeasurementState &sample,
    std::chrono::steady_clock::time_point received)
{
    pm_msgs::msg::LaserMeasurement msg;
    msg.measurement = sample.measurement;
    msg.sequence = sample.sequence;
    msg.status_code = sample.status;
    msg.scanning = sample.scanning;
    msg.message = sample.message;
    const auto local_age = received == std::chrono::steady_clock::time_point{} ? 0 :
        std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - received).count());
    msg.age_ms = static_cast<std::uint32_t>(std::min<std::uint64_t>(0xFFFFFFFFu,
        static_cast<std::uint64_t>(sample.age_ms) + local_age));
    // OPC UA DateTime is 100 ns since 1601; ROS Time uses seconds since 1970.
    const bool timestamp_valid = sample.source_timestamp >= UA_DATETIME_UNIX_EPOCH &&
        sample.source_timestamp <= UA_DATETIME_UNIX_EPOCH +
            static_cast<UA_DateTime>(INT32_MAX) * UA_DATETIME_SEC + UA_DATETIME_SEC - 1;
    if (timestamp_valid)
    {
        const auto ticks = sample.source_timestamp - UA_DATETIME_UNIX_EPOCH;
        msg.source_timestamp.sec = static_cast<std::int32_t>(ticks / UA_DATETIME_SEC);
        msg.source_timestamp.nanosec = static_cast<std::uint32_t>((ticks % UA_DATETIME_SEC) * 100);
    }
    msg.valid = sample.valid() && timestamp_valid && msg.age_ms <= 500 && std::isfinite(msg.measurement);
    if (!msg.valid)
    {
        msg.measurement = std::numeric_limits<double>::quiet_NaN();
        if (msg.status_code == UA_STATUSCODE_GOOD)
        {
            msg.status_code = UA_STATUSCODE_BADNODATA;
            msg.message = timestamp_valid ? "Measurement became stale during ROS delivery" :
                "Measurement timestamp is outside the ROS time range";
        }
    }
    return msg;
}
} // namespace


PMSensorController::PMSensorController()
{
    RCLCPP_INFO(rclcpp::get_logger("PMSensorController"), "PMSensorController instantiated.");
}

controller_interface::return_type PMSensorController::init(
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

controller_interface::CallbackReturn PMSensorController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    m_reference_cube_pub =
        get_node()->create_publisher<std_msgs::msg::Bool>("~/ReferenceCube/Stream", 1000);
    m_laser_pub = get_node()->create_publisher<pm_msgs::msg::LaserMeasurement>("~/Laser/Stream", 10);
    if (!get_node()->has_parameter("laser_opcua_endpoint"))
        get_node()->declare_parameter<std::string>("laser_opcua_endpoint", "");
    if (!get_node()->has_parameter("laser_measurement_timeout_ms"))
        get_node()->declare_parameter<int>("laser_measurement_timeout_ms", 120000);
    if (!get_node()->has_parameter("laser_rpc_timeout_ms"))
        get_node()->declare_parameter<int>("laser_rpc_timeout_ms", 500);
    m_laser_endpoint = get_node()->get_parameter("laser_opcua_endpoint").as_string();
    const auto timeout = get_node()->get_parameter("laser_measurement_timeout_ms").as_int();
    const auto rpc_timeout = get_node()->get_parameter("laser_rpc_timeout_ms").as_int();
    if (timeout < 1 || timeout > 600000 || rpc_timeout < 1 || rpc_timeout > 5000)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Laser timeout must be 1..600000 ms and RPC timeout 1..5000 ms");
        return controller_interface::CallbackReturn::ERROR;
    }
    m_laser_timeout_ms = static_cast<unsigned int>(timeout);
    m_laser_rpc_timeout_ms = static_cast<unsigned int>(rpc_timeout);
    m_force_pub = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/ForceSensor/Stream",
        1000
    );

    m_force_sensor_get_measurement = get_node()->create_service<ForceSensorGetMeasurement>(
        "~/ForceSensor/GetMeasurement",
        [this](
            const ForceSensorGetMeasurement::Request::SharedPtr request,
            ForceSensorGetMeasurement::Response::SharedPtr response
        ) {
            (void)request;
            std::copy_n(
                std::begin(m_force_sensor_measurement),
                m_force_sensor_measurement.size(),
                std::begin(response->data)
            );
        }
    );

    m_force_sensor_bias = get_node()->create_service<ForceSensorBias>(
        "~/ForceSensor/Bias",
        [this](
            const ForceSensorBias::Request::SharedPtr request,
            ForceSensorBias::Response::SharedPtr response
        ) {
            (void)response;
            (void)request;
            m_force_sensor_bias_cmd = true;
        }
    );

    // Humble supports deferred responses: the executor never waits for hardware.
    m_laser_get_measurement = get_node()->create_service<LaserGetMeasurement>(
        "~/Laser/GetMeasurement",
        [this](rclcpp::Service<LaserGetMeasurement>::SharedPtr service,
            std::shared_ptr<rmw_request_id_t> header,
            LaserGetMeasurement::Request::SharedPtr request) {
            LaserGetMeasurement::Response rejected;
            bool accept = false;
            {
                std::lock_guard<std::mutex> lock(m_laser_mutex);
                const auto timeout = request->timeout_ms == 0 ? m_laser_timeout_ms : request->timeout_ms;
                if (!m_laser_running || m_laser_endpoint.empty())
                    rejected.message = "Laser service inactive or laser_opcua_endpoint not configured";
                else if (timeout > 600000)
                    rejected.message = "timeout_ms must be at most 600000";
                else if (m_laser_pending)
                    rejected.message = "A fresh measurement request is already pending";
                else
                {
                    ++m_laser_token;
                    m_laser_pending = true;
                    m_laser_ready = false;
                    m_laser_deadline = Clock::now() + std::chrono::milliseconds(timeout);
                    m_laser_request_header = header;
                    m_laser_response_service = service;
                    m_laser_response = LaserGetMeasurement::Response{};
                    accept = true;
                }
            }
            if (accept) m_laser_condition.notify_all();
            else
            {
                rejected.sample = ToMessage(PMClient::LaserMeasurementState{}, Clock::time_point{});
                service->send_response(*header, rejected);
            }
        });
    m_laser_response_timer = get_node()->create_wall_timer(std::chrono::milliseconds(10),
        [this] { complete_laser_response(); });

    m_reference_cube_state = get_node()->create_service<ReferenceCubeState>(
        "~/ReferenceCube/State",
        [this](
            const ReferenceCubeState::Request::SharedPtr request,
            ReferenceCubeState::Response::SharedPtr response
        ) {
            (void)request;
            response->pressed = m_reference_cube_pressed;
        }
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    if (!m_laser_running.exchange(true)) m_laser_thread = std::thread(&PMSensorController::laser_worker, this);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PMSensorController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    stop_laser_worker();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMSensorController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Force/Bias",
    };
    return config;
}

controller_interface::InterfaceConfiguration
PMSensorController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {
        "Laser/Measurement",
        "Force/X",
        "Force/Y",
        "Force/Z",
        "Force/TX",
        "Force/TY",
        "Force/TZ",
        "ReferenceCube/Pushed",
    };
    return config;
}

controller_interface::return_type
PMSensorController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;

    {
        std_msgs::msg::Bool msg;
        m_reference_cube_pressed = msg.data = static_cast<bool>(state_interfaces_[7].get_value());
        m_reference_cube_pub->publish(msg);
    }

    {
        PMClient::LaserMeasurementState sample;
        Clock::time_point received;
        {
            std::lock_guard<std::mutex> lock(m_laser_mutex);
            sample = m_laser_sample;
            received = m_laser_received;
        }
        m_laser_pub->publish(ToMessage(sample, received));
    }

    {
        std_msgs::msg::Float64MultiArray msg;
        for (auto i = 0; i < 6; i++)
        {
            m_force_sensor_measurement[i] = state_interfaces_[1 + i].get_value();
            msg.data.emplace_back(m_force_sensor_measurement[i]);
        }
        m_force_pub->publish(msg);
    }

    command_interfaces_[0].set_value(static_cast<double>(m_force_sensor_bias_cmd));
    m_force_sensor_bias_cmd = false;

    return controller_interface::return_type::OK;
}

PMSensorController::~PMSensorController()
{
    stop_laser_worker();
}

void PMSensorController::stop_laser_worker()
{
    m_laser_running = false;
    m_laser_condition.notify_all();
    {
        std::lock_guard<std::mutex> lock(m_laser_mutex);
        if (m_laser_pending)
        {
            m_laser_response.success = false;
            m_laser_response.message = "Controller deactivated before measurement completed";
            m_laser_response.sample = ToMessage(PMClient::LaserMeasurementState{}, Clock::time_point{});
            m_laser_ready = true;
        }
        m_laser_sample = PMClient::LaserMeasurementState{};
        m_laser_sample.message = "Controller inactive";
    }
    complete_laser_response();
    if (m_laser_thread.joinable()) m_laser_thread.join();
}

void PMSensorController::complete_laser_response()
{
    rclcpp::Service<LaserGetMeasurement>::SharedPtr service;
    std::shared_ptr<rmw_request_id_t> header;
    LaserGetMeasurement::Response response;
    {
        std::lock_guard<std::mutex> lock(m_laser_mutex);
        if (!m_laser_pending) return;
        if (Clock::now() >= m_laser_deadline)
        {
            m_laser_response = LaserGetMeasurement::Response{};
            m_laser_response.sample = ToMessage(PMClient::LaserMeasurementState{}, Clock::time_point{});
            m_laser_response.sample.status_code = UA_STATUSCODE_BADTIMEOUT;
            m_laser_response.message = "Timed out waiting for a fresh measurement; any scan continues on the server";
            m_laser_ready = true;
        }
        if (!m_laser_ready) return;
        service = m_laser_response_service;
        header = m_laser_request_header;
        response = m_laser_response;
        if (response.success)
        {
            response.sample = ToMessage(m_laser_completion_sample, m_laser_completion_received);
            response.success = response.sample.valid;
            if (!response.success) response.message = response.sample.message;
        }
        m_laser_pending = false;
        m_laser_ready = false;
        m_laser_response_service.reset();
        m_laser_request_header.reset();
    }
    try { if (service && header) service->send_response(*header, response); }
    catch (const std::exception &error)
    { RCLCPP_WARN(get_node()->get_logger(), "Laser response delivery failed: %s", error.what()); }
}

void PMSensorController::laser_worker()
{
    std::unique_ptr<PMClient::Client> client;
    std::uint64_t token = 0;
    PMClient::LaserMeasurementRequest request;
    while (m_laser_running)
    {
        bool pending = false;
        Clock::time_point deadline;
        {
            std::lock_guard<std::mutex> lock(m_laser_mutex);
            pending = m_laser_pending && !m_laser_ready;
            deadline = m_laser_deadline;
            if (token != m_laser_token)
            {
                token = m_laser_token;
                request = PMClient::LaserMeasurementRequest{};
            }
        }
        PMClient::LaserMeasurementState sample;
        bool done = false;
        try
        {
            if (m_laser_endpoint.empty()) throw std::runtime_error("laser_opcua_endpoint is not configured");
            if (!client)
            {
                client = std::make_unique<PMClient::Client>();
                client->set_timeout(m_laser_rpc_timeout_ms);
                client->connect(m_laser_endpoint);
                client->init();
            }
            if (!m_laser_running) break;
            if (!client->get_robot().skills) throw std::runtime_error("OPC UA server has no skills folder");
            auto &skills = *client->get_robot().skills;
            if (pending)
            {
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
                if (remaining <= 0) throw std::runtime_error("Fresh measurement deadline expired");
                const auto budget = static_cast<unsigned int>(std::min<std::int64_t>(m_laser_rpc_timeout_ms, remaining));
                if (request.id == 0)
                    request = skills.request_laser_measurement(static_cast<unsigned int>(remaining), budget);
                const auto poll_remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
                if (poll_remaining <= 0) throw std::runtime_error("Fresh measurement deadline expired");
                sample = skills.laser_measurement_result(request, done,
                    static_cast<unsigned int>(std::min<std::int64_t>(m_laser_rpc_timeout_ms, poll_remaining)));
            }
            else sample = skills.laser_measurement_state(m_laser_rpc_timeout_ms);
        }
        catch (const std::exception &error)
        {
            sample.status = UA_STATUSCODE_BADCOMMUNICATIONERROR;
            sample.message = error.what();
            done = pending;
            client.reset();
        }
        {
            std::lock_guard<std::mutex> lock(m_laser_mutex);
            if (!m_laser_running) break;
            m_laser_sample = sample;
            m_laser_received = Clock::now();
            if (pending && done && m_laser_pending && token == m_laser_token && !m_laser_ready)
            {
                m_laser_completion_sample = sample;
                m_laser_completion_received = m_laser_received;
                m_laser_response.sample = ToMessage(sample, m_laser_received);
                m_laser_response.success = m_laser_response.sample.valid && Clock::now() < deadline;
                m_laser_response.message = sample.message;
                m_laser_ready = true;
            }
        }
        std::unique_lock<std::mutex> lock(m_laser_mutex);
        m_laser_condition.wait_for(lock, std::chrono::milliseconds(100),
            [this, token] { return !m_laser_running || m_laser_token != token; });
    }
}

} // namespace pm_sensor_controller

PLUGINLIB_EXPORT_CLASS(
    pm_sensor_controller::PMSensorController, controller_interface::ControllerInterface
)
