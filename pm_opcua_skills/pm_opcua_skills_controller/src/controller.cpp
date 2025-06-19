
#include "pm_opcua_skills_msgs/srv/dispense.hpp"
// #include "pm_opcua_skills_msgs/srv/force_sensing_move.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "pm_client/client.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#define PI 3.14159265359

using namespace pm_opcua_skills_msgs::srv;
using namespace control_msgs::action;

class Controller : public rclcpp::Node
{
    PMClient::Client m_client;

    rclcpp::Service<Dispense>::SharedPtr m_dispense_service;
    // rclcpp::Service<ForceSensingMove>::SharedPtr m_force_sensing_move_service;

    // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr xyz_joint_client;

    // rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr t_joint_client;

  public:
    Controller() : Node("pm_opcua_skills_controller")
    {
        std::string opcua_server_address = this->declare_parameter<std::string>(
            "opcua_server_address",
            "opc.tcp://PC1M0484-1:4840"
        );
        m_client.connect(opcua_server_address);
        m_client.init();

        m_dispense_service = create_service<Dispense>(
            "~/Dispense",
            [this](
                const Dispense::Request::SharedPtr request,
                Dispense::Response::SharedPtr response
            ) {
                const auto success = m_client.get_robot().skills->dispense(
                    request->time,
                    request->z_height,
                    request->z_move
                );
                response->success = success;
            }
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
//     point.time_from_start = rclcpp::Duration::from_seconds(1.0); // Set the time to reach the goal

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
//     point.time_from_start = rclcpp::Duration::from_seconds(1.0); // Set the time to reach the goal

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
}
;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
