
#include "pm_msgs/srv/laser_line_scan.hpp"
#include "pm_opcua_skills_msgs/srv/dispense.hpp"

// #include "pm_opcua_skills_msgs/srv/force_sensing_move.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "pm_client/client.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#define PI 3.14159265359

using Dispense = pm_opcua_skills_msgs::srv::Dispense;
using LaserLineScan = pm_msgs::srv::LaserLineScan;
using namespace control_msgs::action;

class Controller : public rclcpp::Node
{
    PMClient::Client m_client;
    PMClient::Client m_laser_client;
    rclcpp::CallbackGroup::SharedPtr m_laser_group;
    unsigned int m_scan_timeout_ms = 120000;
    std::string opcua_server_address_;
    rclcpp::Service<Dispense>::SharedPtr m_dispense_service;
    rclcpp::Service<LaserLineScan>::SharedPtr m_laser_line_service;

    // rclcpp::Service<ForceSensingMove>::SharedPtr m_force_sensing_move_service;

    // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    // xyz_joint_client;

    // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr t_joint_client;

    void laser_line_scan_callback(
        const LaserLineScan::Request::SharedPtr request, LaserLineScan::Response::SharedPtr response
    )
    {
        response->success = false;
        response->distances.clear();

        // ROS and OPC UA use UInt32; validate the supported sensor range.
        if (request->length_um < 1 || request->length_um > 1100 || request->interval < 1 ||
            request->interval > std::numeric_limits<unsigned int>::max())
        {
            response->message =
                "length_um must be 1..1100 and interval must be a positive UInt32 (in um)";
            RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
            return;
        }

        const auto result = m_laser_client.get_robot().skills->laser_line_scan(
            static_cast<unsigned int>(request->interval),
            static_cast<unsigned int>(request->length_um),
            opcua_server_address_, m_scan_timeout_ms
        );
        response->success = result.success;
        response->distances = result.distances;
        response->message = result.message;
        response->actual_length_um = result.actual_length_um;
        response->actual_interval_um = result.actual_interval_um;
        response->sample_count = static_cast<std::uint32_t>(result.distances.size());
        response->restoration_verified = result.restoration_verified;
        if (!response->success)
        {
            RCLCPP_ERROR(get_logger(), "Laser line scan failed: %s", response->message.c_str());
        }
    }

  public:
    Controller() : Node("pm_opcua_skills_controller")
    {
        opcua_server_address_ = this->declare_parameter<std::string>(
            "opcua_server_address",
            "opc.tcp://PC1M0484-1:4840"
        );
        m_client.connect(opcua_server_address_);
        m_client.init();
        const auto scan_timeout = declare_parameter<int>("laser_scan_timeout_ms", 120000);
        if (scan_timeout < 1 || scan_timeout > 600000)
            throw std::invalid_argument("laser_scan_timeout_ms must be 1..600000");
        m_scan_timeout_ms = static_cast<unsigned int>(scan_timeout);
        m_laser_client.set_timeout(1000);
        m_laser_client.connect(opcua_server_address_);
        m_laser_client.init();
        m_laser_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        m_dispense_service = create_service<Dispense>(
            "~/Dispense",
            [this](
                const Dispense::Request::SharedPtr request,
                Dispense::Response::SharedPtr response
            ) {
                const auto success = m_client.get_robot().skills->dispense(
                    request->time,
                    request->z_height,
                    request->z_move,
                    opcua_server_address_
                );
                response->success = success;
                if (!success)
                {
                    RCLCPP_ERROR(get_logger(), "Error calling dispense skill");
                }
            }
        );

        m_laser_line_service = create_service<LaserLineScan>(
            "~/LaserLineScan",
            [this](
                const LaserLineScan::Request::SharedPtr request,
                LaserLineScan::Response::SharedPtr response
            ) { laser_line_scan_callback(request, response); },
            rmw_qos_profile_services_default, m_laser_group
        );

        // m_force_sensing_move_service = create_service<ForceSensingMove>(
        //     "~/ForceSensingMove",
        //     [this](
        //         const ForceSensingMove::Request::SharedPtr request,
        //         ForceSensingMove::Response::SharedPtr response
        //     ) {
        //     int start_x = static_cast<int>(request->start_x * 1e6);
        //     int start_y = static_cast<int>(request->start_y * 1e6);
        //     int start_z = static_cast<int>(request->start_z * 1e6);
        //     int start_t = static_cast<int>(request->start_t * 180 / PI);

        //     int target_x = static_cast<int>(request->target_x * 1e6);
        //     int target_y = static_cast<int>(request->target_y * 1e6);
        //     int target_z = static_cast<int>(request->target_z * 1e6);

        //     // RCLCPP_INFO(get_logger(), "Goal send");
        //     // if (!send_goal_xyz(start_x * 1e-6, start_y * 1e-6, start_z * 1e-6))
        //     // {
        //     //     response->success = false;
        //     //     response->error = "Goal sending failed.";
        //     // }

        //     const auto res = m_client.get_robot().skills->force_sensing_move(
        //         start_x,
        //         start_y,
        //         start_z,
        //         start_t,
        //         target_x,
        //         target_y,
        //         target_z,
        //         request->max_fx,
        //         request->max_fy,
        //         request->max_fz,
        //         request->step_size
        //     );

        // RCLCPP_INFO(get_logger(), "Goal send");
        // if (!send_goal_xyz(target_x * 1e-6, target_y * 1e-6, target_z * 1e-6))
        // {
        //     response->success = false;
        //     response->error = "Goal sending failed.";
        // }
        // if (!send_goal_t(start_t))
        // {
        //     response->success = false;
        //     response->error = "Goal sending failed.";
        //             }

        //             response->success = res.success;
        //             response->error = res.error;
        //             response->threshold_exceeded = res.threshold_exceeded;
        // }
        //     );

        //     Initialize the action client
        //     xyz_joint_client =
        //         rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        //             this,
        //             "/pm_robot_xyz_axis_controller/follow_joint_trajectory"
        //         );

        //     // Initialize the action client
        //     t_joint_client =
        //     rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        //         this,
        //         "/pm_robot_t_axis_controller/follow_joint_trajectory"
        //     );

        RCLCPP_INFO(get_logger(), "Controller running...");
    }

    // private : bool
    //           send_goal_xyz(double x_goal, double y_goal, double z_goal)
    // {
    //     // Wait for the action server to be available
    //     if (!xyz_joint_client->wait_for_action_server())
    //     {
    //         RCLCPP_ERROR(get_logger(), "Action server not available.");
    //         return false;
    //     }

    //     // Create a FollowJointTrajectory goal
    //     FollowJointTrajectory::Goal goal;
    //     goal.trajectory.joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"};

    //     // Create a trajectory point with the target joint positions
    //     trajectory_msgs::msg::JointTrajectoryPoint point;
    //     point.positions = {x_goal, y_goal, z_goal};
    //     point.time_from_start = rclcpp::Duration::from_seconds(1.0); // Set the time to reach the
    //     goal

    //     goal.trajectory.points.push_back(point);

    //     // auto send_goal_options =
    //     // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    //     // send_goal_options.goal_response_callback=std:bind(&)

    //     // Send the goal to the action server
    //     auto result_future = xyz_joint_client->async_send_goal(goal);

    //     // Wait for result
    //     // auto status = result_future.wait_for(std::chrono::seconds(30)); does not work yet
    //     return true;
    // }

    // bool send_goal_t(double t_goal)
    // {
    //     if (!t_joint_client->wait_for_action_server())
    //     {
    //         RCLCPP_ERROR(get_logger(), "Action server not available.");
    //         return false;
    //     }

    //     // Create a FollowJointTrajectory goal
    //     FollowJointTrajectory::Goal goal;
    //     goal.trajectory.joint_names = {"T_Axis_Joint"};

    //     // Create a trajectory point with the target joint positions
    //     trajectory_msgs::msg::JointTrajectoryPoint point;
    //     point.positions = {t_goal};
    //     point.time_from_start = rclcpp::Duration::from_seconds(1.0); // Set the time to reach the
    //     goal

    //     goal.trajectory.points.push_back(point);

    //     // auto send_goal_options =
    //     // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    //     // send_goal_options.goal_response_callback=std:bind(&)

    //     // Send the goal to the action server
    //     auto result_future = t_joint_client->async_send_goal(goal);

    //     // Wait for result
    //     // auto status = result_future.wait_for(std::chrono::seconds(30)); does not work yet
    //     return true;
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<Controller>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
    executor.add_node(controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
