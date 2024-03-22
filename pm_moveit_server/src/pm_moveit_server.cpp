#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuple>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include "pm_moveit_interfaces/srv/execute_plan.hpp"

#include "pm_moveit_interfaces/srv/move_relative.hpp"
#include "pm_moveit_interfaces/srv/move_to_pose.hpp"
#include "pm_moveit_interfaces/srv/move_to_frame.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <iomanip>
#include <geometry_msgs/msg/pose.hpp>

Eigen::Affine3d poseMsgToAffine(const geometry_msgs::msg::Pose& pose_msg) {
    // Extract translation
    Eigen::Translation3d translation(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

    // Extract rotation
    Eigen::Quaterniond rotation(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);

    // Construct Affine3d transformation
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() = translation.vector(); // Set translation component
    affine.rotate(rotation);

    return affine;
}


geometry_msgs::msg::Quaternion quaternion_multiply(geometry_msgs::msg::Quaternion q0, geometry_msgs::msg::Quaternion q1){
  // Extract the values from q0
  auto w0 = q0.w;
  auto x0 = q0.x;
  auto y0 = q0.y;
  auto z0 = q0.z;

  // Extract the values from q1
  auto w1 = q1.w;
  auto x1 = q1.x;
  auto y1 = q1.y;
  auto z1 = q1.z;

  // Computer the product of the two quaternions, term by term
  auto q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
  auto q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
  auto q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
  auto q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

  auto result = geometry_msgs::msg::Quaternion();
  result.x = q0q1_x;
  result.y = q0q1_y;
  result.z = q0q1_z;
  result.w = q0q1_w;

  return result;
}


// class PmMoveitServer : public rclcpp::Node
// {
// public:

//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;
  
//   rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service;
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service;
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service;
//   rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service;

//   std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

//   std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   sensor_msgs::msg::JointState::SharedPtr global_joint_state;
//   // create shared pointer to node publisher
//   std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> xyz_trajectory_publisher;
//   std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> t_trajectory_publisher;

//   PmMoveitServer()
//   : Node("pm_moveit_server")
//   {

//     // RCLCPP_INFO(this->get_logger(), "Ready for operation1...");
//     // plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
//     // laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->shared_from_this(), "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
//     // laser_grp_visual_tools->deleteAllMarkers();
//     // laser_grp_visual_tools->loadRemoteControl();
//     // PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(),"robot_description");
//     auto callback_group_re = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//     rclcpp::SubscriptionOptions node_options_1;
//     node_options_1.callback_group = callback_group_re;

//     auto callback_group_me = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     rclcpp::SubscriptionOptions node_options_2;
//     node_options_2.callback_group = callback_group_me;
    
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//     xyz_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_xyz_axis_controller/joint_trajectory", 10);
//     t_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_t_axis_controller/joint_trajectory", 10);
//     auto joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&PmMoveitServer::jointStateCallback, this, std::placeholders::_1),node_options_1);
//     execute_plan_service = this->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&PmMoveitServer::execute_plan, this, std::placeholders::_1, std::placeholders::_2));
//     move_cam_one_service = this->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", std::bind(&PmMoveitServer::move_group_cam1, this, std::placeholders::_1, std::placeholders::_2));
//     move_tool_service = this->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", std::bind(&PmMoveitServer::move_group_tool, this, std::placeholders::_1, std::placeholders::_2));
//     move_laser_service = this->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", std::bind(&PmMoveitServer::move_group_laser, this, std::placeholders::_1, std::placeholders::_2));
//     //RCLCPP_INFO(this->get_logger(), "Ready for operation...");
//   }

// std::tuple<bool, std::vector<std::string>, std::vector<double>> exec_move_group_service(std::string planning_group,
//                                                                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
//                                                                            std::string frame_name,
//                                                                            geometry_msgs::msg::Pose move_to_pose,
//                                                                            geometry_msgs::msg::Vector3 translation,
//                                                                            geometry_msgs::msg::Quaternion rotation,
//                                                                            bool exec_wait_for_user_input,
//                                                                            bool execute)
// {

  
//   std::string endeffector = move_group->getEndEffectorLink();
//   RCLCPP_INFO(this->get_logger(), "Endeffector Link: %s", endeffector.c_str());
  
//   const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState(1.0)->getJointModelGroup(planning_group);
//   RCLCPP_WARN(this->get_logger(), "Test");
//   const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
//   const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();

//   moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//   robot_state->setToDefaultValues();
//   auto state = moveit::core::RobotState(kinematic_model);

//   geometry_msgs::msg::Pose target_pose;
//   std::vector<double> target_joint_values;
//   bool service_success;
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//   geometry_msgs::msg::Quaternion target_rotation;

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Retrieved Rotation x: %f, y: %f, z: %f, w: %f", rotation.x, rotation.y, rotation.z, rotation.w);

//   // if rotation is not specified
//   if (rotation.w == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0)
//   {
//     rotation.w = 1;
//   }

//   // if rotation is not specified
//   if (move_to_pose.orientation.w == 0.0 && move_to_pose.orientation.x == 0.0 && move_to_pose.orientation.y == 0.0 && move_to_pose.orientation.z == 0.0)
//   {
//     move_to_pose.orientation.w = 1;
//   }

//   geometry_msgs::msg::TransformStamped frame_transform;

//   // if target position is (0,0,0) target pose is set the the endeffector; using the translation, this can be used for relative movement
//   if ((move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0) && frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "No pose or frame given! Executing relative movement with: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
//     try
//     {
//       frame_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);
//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
      
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_FATAL(rclcpp::get_logger("pm_moveit"), "Could not transform %s to 'world': %s", endeffector.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }
//   // if frame_name is empty
//   else if (frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame not given. Considering given Pose!");

//     target_pose.position.x = move_to_pose.position.x + translation.x;
//     target_pose.position.y = move_to_pose.position.y + translation.y;
//     target_pose.position.z = move_to_pose.position.z + translation.z;
//     target_pose.orientation = quaternion_multiply(move_to_pose.orientation, rotation);
//   }
//   else
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame given. Ignoring given Pose!");
//     std::string fromFrameRel = frame_name;
//     std::string toFrameRel = "world";
//     try
//     {
//       geometry_msgs::msg::TransformStamped frame_transform;
//       frame_transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
//       // target_pose.orientation.x = 0;
//       // target_pose.orientation.y = 0;
//       // target_pose.orientation.z = 0;
//       // target_pose.orientation.w = 1;
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Calculated Endeffector Pose:");
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X %f", target_pose.position.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y %f", target_pose.position.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z %f", target_pose.position.z);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation W %f", target_pose.orientation.w);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation X %f", target_pose.orientation.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Y %f", target_pose.orientation.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Z %f", target_pose.orientation.z);

//   double timeout = 0.1;
//   // Calculate Inverse Kinematik Solution
//   bool success_found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

//   robot_state->updateLinkTransforms();

//   const Eigen::Isometry3d end_effector_state = robot_state->getGlobalLinkTransform(endeffector);
//   tf2::Transform tf2Transform;
//   //tf2::doTransform(end_effector_state, end_effector_state, planned_eef_pose)
//   tf2::convert(end_effector_state,tf2Transform);
//   tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();

//   // This is the same as the calculated Endeffector Pose
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());

//   bool success_calculate_plan = false;

//   if (exec_wait_for_user_input)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "YES");
//   }

//   if (success_found_ik)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK solution found!");
//     robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);

//     for (std::size_t i = 0; i < joint_names.size(); ++i)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %f", joint_names[i].c_str(), target_joint_values[i]);
//     }
//     move_group->setPlanningTime(20);
//     //move_group->setGoalPositionTolerance(1e-9); // 10 nm    
//     move_group->setGoalPositionTolerance(0.000000001); // 1 nm    
//     move_group->setStartStateToCurrentState();
//     move_group->setJointValueTarget(target_joint_values);
//     //move_group->setNumPlanningAttempts(100);
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Test 1");
//     success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Test 11");
//     if (success_calculate_plan)
//     {

//       service_success = true;
//     }
//     else
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
//       service_success = false;
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Did not find IK solution");
//     service_success = false;
//   }
//   //laser_grp_visual_tools->deleteAllMarkers();
//   //auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
//   //laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
//   //laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
//   ////laser_grp_visual_tools->prompt("next step");
//   //laser_grp_visual_tools->trigger();
//   RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Test 111");
//   // Execute the plan
//   if (success_calculate_plan && execute)
//   {
//     move_group->execute(*plan);
//     // Checking delta;
//     try
//     {
//       float delta = 0.0005;  // in meters
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Starting moving..");
//       while (check_goal_reached(joint_names, target_joint_values, 0.0005, 0.01) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }
//       auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//       trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

//       trajectory_msgs::msg::JointTrajectoryPoint point;
//       point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]};  // Specify joint positions
//       point.velocities = {0.0, 0.0, 0.0}; // Specify joint velocities
//       point.accelerations = {0.0, 0.0, 0.0}; // Specify joint accelerations
//       point.time_from_start.sec = 0.5; // Specify duration
//       trajectory_msg->points.push_back(point);
//       xyz_trajectory_publisher->publish(*trajectory_msg);

//       // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
//       if (planning_group == "PM_Robot_Tool_TCP")
//       {
//         auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//         t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
//         trajectory_msgs::msg::JointTrajectoryPoint t_point;
//         t_point.positions = {target_joint_values[3]};  // Specify joint positions
//         t_point.velocities = {0.0}; // Specify joint velocities
//         t_point.accelerations = {0.0}; // Specify joint accelerations
//         t_point.time_from_start.sec = 0.5; // Specify duration
//         t_trajectory_msg->points.push_back(t_point);
//         t_trajectory_publisher->publish(*t_trajectory_msg);
//       }

//       while (check_goal_reached(joint_names, target_joint_values, 0.0000001, 0.0001) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }

//       geometry_msgs::msg::TransformStamped moved_to_transform;
//       geometry_msgs::msg::Pose moved_to_pose;

//       moved_to_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);

//       moved_to_pose.position.x = moved_to_transform.transform.translation.x;
//       moved_to_pose.position.y = moved_to_transform.transform.translation.y;
//       moved_to_pose.position.z = moved_to_transform.transform.translation.z;

//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Moved to Endeffector Pose: ");
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "X: %f", moved_to_pose.position.x);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Y: %f", moved_to_pose.position.y);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Z: %f", moved_to_pose.position.z);
      
//       // auto this_pose = move_group->getCurrentPose(endeffector);

//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X4 %f", this_pose.pose.position.x);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y4 %f", this_pose.pose.position.y);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z4 %f", this_pose.pose.position.z);

//       // double deltaX = moved_to_pose.position.x - target_pose.position.x;
//       // double deltaY = moved_to_pose.position.y - target_pose.position.y;
//       // double deltaZ = moved_to_pose.position.z - target_pose.position.z;
      
//       double deltaX = endeffector_pose_planed.getX() - moved_to_pose.position.x;
//       double deltaY = endeffector_pose_planed.getY() - moved_to_pose.position.y;
//       double deltaZ = endeffector_pose_planed.getZ() - moved_to_pose.position.z;
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Pose Deltas: ");
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "X: %f um", deltaX * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Y: %f um", deltaY * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Z: %f um", deltaZ * 1000000);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Transform Error !!!");
//     }
//   }
//   else
//   {
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan not executed!");
//   }
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
//   return {service_success, joint_names, target_joint_values};
// }


//   void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//     //Process the received joint state message
//     for (size_t i = 0; i < msg->name.size(); ++i) {
//         std::cout << "Joint Name: " << msg->name[i] << ", Position: " << msg->position[i] << std::endl;
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Joint Name: %s, Value: %f",msg->name[i].c_str(), msg->position[i]);
//     }
//     global_joint_state = msg;
//   }

//   void move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Response> response)
//   {
//     RCLCPP_INFO(this->get_logger(), "Waiting for next command...");
//     auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Cam1_TCP",
//                                                          Cam1_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//     response->success = success;
//     response->joint_names = joint_names;
//     std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//     response->joint_values = joint_values_float;

//     return;
//   }

//   void move_group_tool(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Response> response)
//   {

//     auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Tool_TCP",
//                                                          tool_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//     response->success = success;
//     response->joint_names = joint_names;
//     std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//     response->joint_values = joint_values_float;

//     return;
//   }

//   void move_group_laser(const std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Request> request,
//                       std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Response> response)
//   {

//     auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Laser_TCP",
//                                                          laser_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//     response->success = success;
//     response->joint_names = joint_names;
//     std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//     response->joint_values = joint_values_float;

//     return;
//   }

//   bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
//                   std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
//   {
//     if (request->run){
//       RCLCPP_INFO(this->get_logger(), "Yes");
//       response->success = true;
//     }

//     return true;
//   }

//   bool check_goal_reached(std::vector<std::string> target_joints, std::vector<double> target_joint_values, float delta_trans, float delta_rot)
//   {
//     float delta_value;
//     for (size_t i = 0; i < target_joints.size(); i++)
//     {
//       if (target_joints[i] == "T_Axis_Joint")
//       {
//         // This means rotation
//         delta_value = delta_rot;
//       }
//       else
//       {
//         // This means translation
//         delta_value = delta_trans;
//       }

//       // find joint in joint state
//       auto it = std::find(global_joint_state->name.begin(), global_joint_state->name.end(), target_joints[i]);
//       if (it == global_joint_state->name.end()) {
//           // Joint not found
//           // Handle error or return false
//           return false;
//       }
//       int current_joint_index = std::distance(global_joint_state->name.begin(), it);
//       float current_joint_value = global_joint_state->position[current_joint_index];
//       float differrence = std::abs(current_joint_value - target_joint_values[i]);
      
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint: %s, Target: %f, Current: %f, Delta: %f", target_joints[i].c_str(), target_joint_values[i], current_joint_value, differrence);
      
      
//       if (differrence > delta_value)
//       {
//         return false;
//       }
//     }    
//     return true;
//   }
// };


// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   auto pm_moveit_server_node = std::make_shared<PmMoveitServer>();

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(pm_moveit_server_node);
//   //executor.spin_once();
//   pm_moveit_server_node->PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node,"robot_description");
//   pm_moveit_server_node->laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
//   pm_moveit_server_node->Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
//   pm_moveit_server_node->tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
//   pm_moveit_server_node->laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,pm_moveit_server_node->laser_move_group->getRobotModel());
  
//   pm_moveit_server_node->laser_grp_visual_tools->deleteAllMarkers();
//   pm_moveit_server_node->laser_grp_visual_tools->loadRemoteControl();
//   //Dont know why this has to be called at this point, but otherwith the service callbacks get stuck.
//   auto current_state_cam1 = pm_moveit_server_node->Cam1_move_group->getCurrentState(1.0);
//   auto current_state_tool = pm_moveit_server_node->tool_move_group->getCurrentState(1.0);
//   auto current_state_laser = pm_moveit_server_node->laser_move_group->getCurrentState(1.0);
//   RCLCPP_INFO(pm_moveit_server_node->get_logger(), "Ready for operation1...");




//   //auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }












// Global Variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;

std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
sensor_msgs::msg::JointState::SharedPtr global_joint_state;
// create shared pointer to node publisher
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> xyz_trajectory_publisher;
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> t_trajectory_publisher;
std::string default_endeffector_string;
std::string global_return_massage;

void log_pose(std::string pose_text, geometry_msgs::msg::Pose pose){
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), pose_text.c_str());
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position X %.9f", pose.position.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position Y %.9f", pose.position.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position Z %.9f", pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation W %f", pose.orientation.w);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation X %f", pose.orientation.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation Y %f", pose.orientation.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation Z %f", pose.orientation.z);
}

bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
                  std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
{
  if (request->run){
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Yes");
    response->success = true;
  }

  return true;
}

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Process the received joint state message
    // For example, print the names and positions of the joints
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::cout << "Joint Name: " << msg->name[i] << ", Position: " << msg->position[i] << std::endl;
        //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Joint Name: %s, Value: %f",msg->name[i].c_str(), msg->position[i]);
    }
    global_joint_state = msg;
}


bool check_goal_reached(std::vector<std::string> target_joints, std::vector<double> target_joint_values, float delta_trans, float delta_rot)
{
  float delta_value;
  for (size_t i = 0; i < target_joints.size(); i++)
  {
    if (target_joints[i] == "T_Axis_Joint")
    {
      // This means rotation
      delta_value = delta_rot;
    }
    else
    {
      // This means translation
      delta_value = delta_trans;
    }

    // find joint in joint state
    auto it = std::find(global_joint_state->name.begin(), global_joint_state->name.end(), target_joints[i]);
    if (it == global_joint_state->name.end()) {
        // Joint not found
        // Handle error or return false
        return false;
    }
    int current_joint_index = std::distance(global_joint_state->name.begin(), it);
    float current_joint_value = global_joint_state->position[current_joint_index];
    float differrence = std::abs(current_joint_value - target_joint_values[i]);
    
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint: %s, Target: %.9f, Current: %.9f, Delta: %.9f", target_joints[i].c_str(), target_joint_values[i], current_joint_value, differrence);
    
    
    if (differrence > delta_value)
    {
      return false;
    }
  }    
  return true;
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> calculate_IK(std::string planning_group, std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group, geometry_msgs::msg::Pose target_pose){
  std::vector<double> target_joint_values;
  std::vector<double> min_joint_values;
  std::vector<double> max_joint_values;
  std::string endeffector = move_group->getEndEffectorLink();
  const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState(1.0)->getJointModelGroup(planning_group);
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));


  // There is an issue when setting the target pose to a pose that is in range of 10 um. 
  // For a reason i dont know, the IK solution returns the current joint values
  // To account for that the robot state is set to the max joint value before setting fromIK
  // Please note that this is a workaround and a better solution should be found for this issue

  // Get bounds of joints
  moveit::core::JointBoundsVector joint_bounds = joint_model_group->getActiveJointModelsBounds();
  
  //get joint limits
  for (size_t i = 0; i < joint_bounds.size(); i++)
  {
    min_joint_values.push_back(joint_bounds[i][0][0].min_position_);
    max_joint_values.push_back(joint_bounds[i][0][0].max_position_);

    RCLCPP_DEBUG(rclcpp::get_logger("pm_moveit"), " Lower: %.9f, Upper: %.9f", joint_bounds[i][0][0].min_position_, joint_bounds[i][0][0].max_position_);
  }
  

  robot_state->setJointGroupPositions(joint_model_group, max_joint_values);
  double timeout = 0.5;
  bool success_found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

  // Eigen::Isometry3d pose_eigen;
  // tf2::convert(target_pose, pose_eigen);
  // std::vector<double> consitency_limits;
  // consitency_limits = {0.0000001, 0.0000001, 0.0000001};
  // bool success_found_ik = robot_state->setFromIK(joint_model_group, pose_eigen, endeffector, consitency_limits, timeout);

  if (success_found_ik)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "IK solution found!");
    robot_state->updateLinkTransforms();
    const Eigen::Isometry3d end_effector_state = robot_state->getGlobalLinkTransform(endeffector);
    tf2::Transform tf2Transform;
    tf2::convert(end_effector_state,tf2Transform);
    //tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();
    // This is the same as the calculated Endeffector Pose
    //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
    //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
    //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());
    robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %.9f", joint_names[i].c_str(), target_joint_values[i]);
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Did not find IK solution");
  }
  return std::make_tuple(success_found_ik, joint_names, target_joint_values);

}

std::tuple<bool, geometry_msgs::msg::Pose> get_pose_of_frame(std::string frame_name)
{
  bool get_pose_success = true;
  geometry_msgs::msg::Pose pose;
  std::string fromFrameRel = frame_name;
  std::string toFrameRel = "world";
  try
  {
    geometry_msgs::msg::TransformStamped frame_transform;
    frame_transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

    pose.position.x = frame_transform.transform.translation.x;
    pose.position.y = frame_transform.transform.translation.y;
    pose.position.z = frame_transform.transform.translation.z;
    pose.orientation = frame_transform.transform.rotation;
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    get_pose_success = false;
  }
  return std::make_tuple(get_pose_success, pose);
}


std::tuple<bool, geometry_msgs::msg::TransformStamped> get_pose_of_frame_in_frame(std::string toFrame, std::string fromFrame)
{
  bool get_pose_success = true;
  geometry_msgs::msg::TransformStamped frame_transform;
  try
  {
    frame_transform = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);

  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrame.c_str(), fromFrame.c_str(), ex.what());
    get_pose_success = false;
  }
  return std::make_tuple(get_pose_success, frame_transform);
}

geometry_msgs::msg::Pose add_translation_rotation_to_pose(geometry_msgs::msg::Pose pose, const geometry_msgs::msg::Vector3 translation, geometry_msgs::msg::Quaternion rotation)
{
  //geometry_msgs::msg::TransformStamped frame_world_transform;
  //tf2::fromMsg(pose, frame_world_transform);
  //geometry_msgs::msg::TransformStamped rel_transform;
  //rel_transform.transform.translation = translation;
  //rel_transform.transform.rotation = rotation;
  //rel_transform.transform.rotation = quaternion_multiply(pose.orientation, rotation);
  tf2::Transform transform_1;
  transform_1.setOrigin(tf2::Vector3( pose.position.x,
                                      pose.position.y,
                                      pose.position.z));
  transform_1.setRotation(tf2::Quaternion(  pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w));

  tf2::Transform transform_2;
  transform_2.setOrigin(tf2::Vector3(translation.x, translation.y, translation.z));
  transform_2.setRotation(tf2::Quaternion(rotation.x,
                                          rotation.y,
                                          rotation.z,
                                            rotation.w));

  tf2::Transform transform_res;
  transform_res.mult(transform_1, transform_2);
  auto vector = transform_res.getOrigin();
  pose.position.x = vector.x();
  pose.position.y = vector.y();
  pose.position.z = vector.z();
  auto quat = transform_res.getRotation();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  //tf2::fromMsg(transform_res, pose);
  return pose;
}

geometry_msgs::msg::Pose get_pose_endeffector_override(std::string initial_endeffector_frame,std::string endeffector_override_frame, geometry_msgs::msg::Pose initial_endeffector_pose)
{
  bool success_frame;
  geometry_msgs::msg::Pose pose_rel;
  geometry_msgs::msg::TransformStamped rel_transform;
  std::tie(success_frame, rel_transform) = get_pose_of_frame_in_frame(endeffector_override_frame,initial_endeffector_frame);
  
  if (!success_frame)
  {
    return initial_endeffector_pose;
  }

  tf2::Transform transform_1;
  transform_1.setOrigin(tf2::Vector3( initial_endeffector_pose.position.x,
                                      initial_endeffector_pose.position.y,
                                      initial_endeffector_pose.position.z));
  transform_1.setRotation(tf2::Quaternion(  initial_endeffector_pose.orientation.x,
                                            initial_endeffector_pose.orientation.y,
                                            initial_endeffector_pose.orientation.z,
                                            initial_endeffector_pose.orientation.w));

  tf2::Transform transform_2;
  transform_2.setOrigin(tf2::Vector3(rel_transform.transform.translation.x, rel_transform.transform.translation.y, rel_transform.transform.translation.z));
  transform_2.setRotation(tf2::Quaternion(rel_transform.transform.rotation.x,
                                          rel_transform.transform.rotation.y,
                                          rel_transform.transform.rotation.z,
                                          rel_transform.transform.rotation.w));

  geometry_msgs::msg::Pose pose;
  tf2::Transform transform_res;

  transform_res.mult(transform_1, transform_2);
  auto vector = transform_res.getOrigin();
  pose.position.x = vector.x();
  pose.position.y = vector.y();
  pose.position.z = vector.z();
  auto quat = transform_res.getRotation();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  //tf2::fromMsg(transform_res, pose);
  return pose;
}

geometry_msgs::msg::Quaternion check_rotation(geometry_msgs::msg::Quaternion rotation)
{
  if (rotation.w == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0)
  {
    rotation.w = 1;
  }
  return rotation;
}

bool set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group, 
                    std::vector<double> target_joint_values,    
                    bool execute_movement)
 {
  bool success_calculate_plan = false;
  move_group->setPlanningTime(20);
  move_group->setStartStateToCurrentState();

  move_group->setGoalJointTolerance(1e-9);

  move_group->setJointValueTarget(target_joint_values);
  move_group->setNumPlanningAttempts(100);
  move_group->setReplanAttempts(10000);
  success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_calculate_plan)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
    return false;
  }

  //laser_grp_visual_tools->deleteAllMarkers();
  //auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
  //laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  //laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
  //laser_grp_visual_tools->prompt("next step");
  //laser_grp_visual_tools->trigger();

  // Execute the plan
  if (success_calculate_plan && execute_movement)
  {
    move_group->execute(*plan);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan calculated successfull, but not execution was not demanded!");
  }
  return true;
  }

void publish_target_joint_trajectory_xyzt(std::string planning_group, std::vector<double> target_joint_values)
  {
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]};  // Specify joint positions
  point.velocities = {0.0, 0.0, 0.0}; // Specify joint velocities
  point.accelerations = {0.0, 0.0, 0.0}; // Specify joint accelerations
  point.time_from_start.sec = 0.5; // Specify duration
  trajectory_msg->points.push_back(point);
  xyz_trajectory_publisher->publish(*trajectory_msg);

  // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
  if (planning_group == "PM_Robot_Tool_TCP")
    {
    auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
    trajectory_msgs::msg::JointTrajectoryPoint t_point;
    t_point.positions = {target_joint_values[3]};  // Specify joint positions
    t_point.velocities = {0.0}; // Specify joint velocities
    t_point.accelerations = {0.0}; // Specify joint accelerations
    t_point.time_from_start.sec = 0.5; // Specify duration
    t_trajectory_msg->points.push_back(t_point);
    t_trajectory_publisher->publish(*t_trajectory_msg);
    }
  }

void wait_for_movement_to_finish(std::vector<std::string> joint_names, std::vector<double> target_joint_values, float lateral_tolerance, float angular_tolerance)
{
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
  while (check_goal_reached(joint_names, target_joint_values, lateral_tolerance, angular_tolerance) == false)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void log_target_pose_delta(std::string endeffector, geometry_msgs::msg::Pose target_pose)
{
  geometry_msgs::msg::Pose moved_to_pose;
  bool frame_success;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  std::tie(frame_success, moved_to_pose) = get_pose_of_frame(endeffector);

  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Moved to Endeffector Pose: ");
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "X: %f", moved_to_pose.position.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Y: %f", moved_to_pose.position.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Z: %f", moved_to_pose.position.z);
    
  double deltaX = target_pose.position.x - moved_to_pose.position.x;
  double deltaY = target_pose.position.y - moved_to_pose.position.y;
  double deltaZ = target_pose.position.z - moved_to_pose.position.z;
  double deltaOrientW = target_pose.orientation.w - moved_to_pose.orientation.w;
  double deltaOrientX = target_pose.orientation.x - moved_to_pose.orientation.x;
  double deltaOrientY = target_pose.orientation.y - moved_to_pose.orientation.y;
  double deltaOrientZ = target_pose.orientation.z - moved_to_pose.orientation.z;

  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Pose Deltas: ");
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "X: %f um", deltaX * 1000000);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Y: %f um", deltaY * 1000000);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Z: %f um", deltaZ * 1000000);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Q_w: %f", deltaOrientW);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Q_x: %f", deltaOrientX);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Q_y: %f", deltaOrientY);
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Q_z: %f", deltaOrientZ);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> move_group_relative(std::string planning_group,
                                                                           std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                           geometry_msgs::msg::Vector3 translation,
                                                                           geometry_msgs::msg::Quaternion rotation,
                                                                           bool execute_movement)
{

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  //bool extract_frame_success;
  // Get the target_pose
  auto [extract_frame_success, target_pose] = get_pose_of_frame(endeffector);

  // This should normaly not happen, because the searched frame is the endeffector, which must exist
  if (!extract_frame_success)
  {
    // If frame is not found, return false
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Specified frame not found!");
    return std::make_tuple(false, joint_names, target_joint_values);
  }

  // Check if rotation is valid and set to default if not
  rotation = check_rotation(rotation);

  target_pose = add_translation_rotation_to_pose(target_pose, translation, rotation);
  log_pose("Calculated Endeffector Pose: ", target_pose);

  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  if (!success_ik)
  {
    return {false, joint_names, target_joint_values};
  }
  bool move_success = set_move_group(move_group, target_joint_values, execute_movement);

  if (!move_success || !execute_movement)
  {
    return {move_success, joint_names, target_joint_values};
  }

  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-7;
  float angular_tolerance_fine = 0.0001;

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  publish_target_joint_trajectory_xyzt(planning_group, target_joint_values);
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
  return std::make_tuple(move_success, joint_names, target_joint_values);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> move_group_to_frame(std::string planning_group,
                                                                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                            std::string endeffector_frame_override,
                                                                            std::string target_frame,
                                                                            geometry_msgs::msg::Vector3 translation,
                                                                            geometry_msgs::msg::Quaternion rotation,
                                                                            bool execute_movement)
{

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  //bool extract_frame_success;
  // Get the target_pose
  auto [extract_frame_success, target_pose] = get_pose_of_frame(target_frame);

  // This should normaly not happen, because the searched frame is the endeffector, which must exist
  if (!extract_frame_success)
  {
    // If frame is not found, return false
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Specified frame not found!");
    return std::make_tuple(false, joint_names, target_joint_values);
  }

  if (endeffector_frame_override != default_endeffector_string)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Overriding Endeffector Frame to: %s", endeffector_frame_override.c_str());
    target_pose = get_pose_endeffector_override(endeffector,endeffector_frame_override,target_pose);
  }

  // Check if rotation is valid and set to default if not
  rotation = check_rotation(rotation);

  target_pose = add_translation_rotation_to_pose(target_pose, translation, rotation);
  log_pose("Calculated Endeffector Pose: ", target_pose);

  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  if (!success_ik)
  {
    return {false, joint_names, target_joint_values};
  }
  bool move_success = set_move_group(move_group, target_joint_values, execute_movement);

  if (!move_success || !execute_movement)
  {
    return {move_success, joint_names, target_joint_values};
  }

  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-7;
  float angular_tolerance_fine = 0.0001;

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  publish_target_joint_trajectory_xyzt(planning_group, target_joint_values);
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");

  return std::make_tuple(move_success, joint_names, target_joint_values);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> move_group_to_pose(std::string planning_group,
                                                                           std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                           geometry_msgs::msg::Pose target_pose,
                                                                           std::string endeffector_frame_override,
                                                                           bool execute_movement)
{
  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  // Get the target_pose

  if (endeffector_frame_override != default_endeffector_string)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Overriding Endeffector Frame to: %s", endeffector_frame_override.c_str());
    target_pose = get_pose_endeffector_override(endeffector,endeffector_frame_override,target_pose);
  }

  log_pose("Calculated Endeffector Pose: ", target_pose);

  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  if (!success_ik)
  {
    return {false, joint_names, target_joint_values};
  }
  bool move_success = set_move_group(move_group, target_joint_values, execute_movement);

  if (!move_success || !execute_movement)
  {
    return {move_success, joint_names, target_joint_values};
  }

  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-7;
  float angular_tolerance_fine = 0.0001;

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  publish_target_joint_trajectory_xyzt(planning_group, target_joint_values);
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");

  return std::make_tuple(move_success, joint_names, target_joint_values);
}



// std::tuple<bool, std::vector<std::string>, std::vector<double>> exec_move_group_service(std::string planning_group,
//                                                                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
//                                                                            std::string frame_name,
//                                                                            geometry_msgs::msg::Pose move_to_pose,
//                                                                            geometry_msgs::msg::Vector3 translation,
//                                                                            geometry_msgs::msg::Quaternion rotation,
//                                                                            bool exec_wait_for_user_input,
//                                                                            bool execute)
// {
//   std::string endeffector = move_group->getEndEffectorLink();
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Endeffector Link: %s", endeffector.c_str());

//   const moveit::core::JointModelGroup *joint_model_group = move_group->getCurrentState(1.0)->getJointModelGroup(planning_group);

//   const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
//   const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();

//   moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
//   robot_state->setToDefaultValues();
//   auto state = moveit::core::RobotState(kinematic_model);

//   geometry_msgs::msg::Pose target_pose;
//   std::vector<double> target_joint_values;
//   bool service_success;

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

//   geometry_msgs::msg::Quaternion target_rotation;

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Retrieved Rotation x: %f, y: %f, z: %f, w: %f", rotation.x, rotation.y, rotation.z, rotation.w);

//   // if rotation is not specified
//   if (rotation.w == 0.0 && rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0)
//   {
//     rotation.w = 1;
//   }

//   // if rotation is not specified
//   if (move_to_pose.orientation.w == 0.0 && move_to_pose.orientation.x == 0.0 && move_to_pose.orientation.y == 0.0 && move_to_pose.orientation.z == 0.0)
//   {
//     move_to_pose.orientation.w = 1;
//   }

//   geometry_msgs::msg::TransformStamped frame_transform;

//   // if target position is (0,0,0) target pose is set the the endeffector; using the translation, this can be used for relative movement
//   if ((move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0 && move_to_pose.position.x == 0.0) && frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "No pose or frame given! Executing relative movement with: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
//     try
//     {
//       frame_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);
//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_FATAL(rclcpp::get_logger("pm_moveit"), "Could not transform %s to 'world': %s", endeffector.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }
//   // if frame_name is empty
//   else if (frame_name == "")
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame not given. Considering given Pose!");

//     target_pose.position.x = move_to_pose.position.x + translation.x;
//     target_pose.position.y = move_to_pose.position.y + translation.y;
//     target_pose.position.z = move_to_pose.position.z + translation.z;
//     target_pose.orientation = quaternion_multiply(move_to_pose.orientation, rotation);
//   }
//   else
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "TF frame given. Ignoring given Pose!");
//     std::string fromFrameRel = frame_name;
//     std::string toFrameRel = "world";
//     try
//     {
//       geometry_msgs::msg::TransformStamped frame_transform;
//       frame_transform = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

//       target_pose.position.x = frame_transform.transform.translation.x + translation.x;
//       target_pose.position.y = frame_transform.transform.translation.y + translation.y;
//       target_pose.position.z = frame_transform.transform.translation.z + translation.z;
//       target_pose.orientation = quaternion_multiply(frame_transform.transform.rotation, rotation);
//       // target_pose.orientation.x = 0;
//       // target_pose.orientation.y = 0;
//       // target_pose.orientation.z = 0;
//       // target_pose.orientation.w = 1;
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//       service_success = false;
//       return {service_success, joint_names, target_joint_values};
//     }
//   }

//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Calculated Endeffector Pose:");
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X %f", target_pose.position.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y %f", target_pose.position.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z %f", target_pose.position.z);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation W %f", target_pose.orientation.w);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation X %f", target_pose.orientation.x);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Y %f", target_pose.orientation.y);
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Orientation Z %f", target_pose.orientation.z);

//   double timeout = 0.1;
//   // Calculate Inverse Kinematik Solution
//   bool success_found_ik = robot_state->setFromIK(joint_model_group, target_pose, timeout);

//   robot_state->updateLinkTransforms();

//   const Eigen::Isometry3d end_effector_state = robot_state->getGlobalLinkTransform(endeffector);
//   tf2::Transform tf2Transform;
//   //tf2::doTransform(end_effector_state, end_effector_state, planned_eef_pose)
//   tf2::convert(end_effector_state,tf2Transform);
//   tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();

//   // This is the same as the calculated Endeffector Pose
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
//   //RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());

//   bool success_calculate_plan = false;

//   if (exec_wait_for_user_input)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "YES");
//   }

//   if (success_found_ik)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK solution found!");
//     robot_state->copyJointGroupPositions(joint_model_group, target_joint_values);

//     for (std::size_t i = 0; i < joint_names.size(); ++i)
//     {
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Joint Values for %s: %f", joint_names[i].c_str(), target_joint_values[i]);
//     }
//     move_group->setPlanningTime(20);
//     //move_group->setGoalPositionTolerance(1e-9); // 10 nm    
//     move_group->setGoalPositionTolerance(0.000000001); // 1 nm    
//     move_group->setStartStateToCurrentState();
//     move_group->setJointValueTarget(target_joint_values);
//     move_group->setNumPlanningAttempts(100);

//     success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success_calculate_plan)
//     {

//       service_success = true;
//     }
//     else
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
//       service_success = false;
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Did not find IK solution");
//     service_success = false;
//   }
//   laser_grp_visual_tools->deleteAllMarkers();
//   auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
//   laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
//   laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
//   //laser_grp_visual_tools->prompt("next step");
//   laser_grp_visual_tools->trigger();

//   // Execute the plan
//   if (success_calculate_plan && execute)
//   {
//     move_group->execute(*plan);
//     // Checking delta;
//     try
//     {
//       while (check_goal_reached(joint_names, target_joint_values, 0.0005, 0.01) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }
//       auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//       trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

//       trajectory_msgs::msg::JointTrajectoryPoint point;
//       point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]};  // Specify joint positions
//       point.velocities = {0.0, 0.0, 0.0}; // Specify joint velocities
//       point.accelerations = {0.0, 0.0, 0.0}; // Specify joint accelerations
//       point.time_from_start.sec = 0.5; // Specify duration
//       trajectory_msg->points.push_back(point);
//       xyz_trajectory_publisher->publish(*trajectory_msg);

//       // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
//       if (planning_group == "PM_Robot_Tool_TCP")
//       {
//         auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
//         t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
//         trajectory_msgs::msg::JointTrajectoryPoint t_point;
//         t_point.positions = {target_joint_values[3]};  // Specify joint positions
//         t_point.velocities = {0.0}; // Specify joint velocities
//         t_point.accelerations = {0.0}; // Specify joint accelerations
//         t_point.time_from_start.sec = 0.5; // Specify duration
//         t_trajectory_msg->points.push_back(t_point);
//         t_trajectory_publisher->publish(*t_trajectory_msg);
//       }

//       while (check_goal_reached(joint_names, target_joint_values, 0.0000001, 0.0001) == false)
//       {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
//       }

//       geometry_msgs::msg::TransformStamped moved_to_transform;
//       geometry_msgs::msg::Pose moved_to_pose;

//       moved_to_transform = tf_buffer_->lookupTransform("world", endeffector, tf2::TimePointZero);

//       moved_to_pose.position.x = moved_to_transform.transform.translation.x;
//       moved_to_pose.position.y = moved_to_transform.transform.translation.y;
//       moved_to_pose.position.z = moved_to_transform.transform.translation.z;

//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Moved to Endeffector Pose: ");
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "X: %f", moved_to_pose.position.x);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Y: %f", moved_to_pose.position.y);
//       RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Z: %f", moved_to_pose.position.z);
      
//       // auto this_pose = move_group->getCurrentPose(endeffector);

//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose X4 %f", this_pose.pose.position.x);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Y4 %f", this_pose.pose.position.y);
//       // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Pose Z4 %f", this_pose.pose.position.z);

//       // double deltaX = moved_to_pose.position.x - target_pose.position.x;
//       // double deltaY = moved_to_pose.position.y - target_pose.position.y;
//       // double deltaZ = moved_to_pose.position.z - target_pose.position.z;
      
//       double deltaX = endeffector_pose_planed.getX() - moved_to_pose.position.x;
//       double deltaY = endeffector_pose_planed.getY() - moved_to_pose.position.y;
//       double deltaZ = endeffector_pose_planed.getZ() - moved_to_pose.position.z;
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Pose Deltas: ");
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "X: %f um", deltaX * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Y: %f um", deltaY * 1000000);
//       RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Z: %f um", deltaZ * 1000000);
//     }
//     catch (const tf2::TransformException &ex)
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Transform Error !!!");
//     }
//   }
//   else
//   {
//     RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan not executed!");
//   }
//   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
//   return {service_success, joint_names, target_joint_values};
// }




// void move_group_cam1(const std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveCam1TcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Cam1_TCP",
//                                                          Cam1_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }

// void move_group_tool(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Request> request,
//                      std::shared_ptr<pm_moveit_interfaces::srv::MoveToolTcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Tool_TCP",
//                                                          tool_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }

// void move_group_laser(const std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Request> request,
//                       std::shared_ptr<pm_moveit_interfaces::srv::MoveLaserTcpTo::Response> response)
// {

//   auto [success, joint_names, joint_values] = exec_move_group_service("PM_Robot_Laser_TCP",
//                                                          laser_move_group,
//                                                          request->frame_name,
//                                                          request->move_to_pose,
//                                                          request->translation,
//                                                          request->rotation,
//                                                          request->exec_wait_for_user_input,
//                                                          request->execute);

//   response->success = success;
//   response->joint_names = joint_names;
//   std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
//   response->joint_values = joint_values_float;

//   return;
// }


// RELATIVE MOVEMENT
void move_cam1_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_relative("PM_Robot_Cam1_TCP",
                                                         Cam1_move_group,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_tool_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_relative("PM_Robot_Tool_TCP",
                                                         tool_move_group,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_laser_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_relative("PM_Robot_Laser_TCP",
                                                         laser_move_group,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

// ABSOLUTE MOVEMENT
void move_cam_one_to_pose(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_pose("PM_Robot_Cam1_TCP",
                                                         Cam1_move_group,
                                                         request->move_to_pose,
                                                         request->endeffector_frame_override,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_tool_to_pose(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_pose("PM_Robot_Tool_TCP",
                                                         tool_move_group,
                                                         request->move_to_pose,
                                                         request->endeffector_frame_override,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_laser_to_pose(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_pose("PM_Robot_Laser_TCP",
                                                         laser_move_group,
                                                         request->move_to_pose,
                                                         request->endeffector_frame_override,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

// ABSOLUTE MOVEMENT TO FRAME
void move_cam_one_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_frame("PM_Robot_Cam1_TCP",
                                                         Cam1_move_group,
                                                         request->endeffector_frame_override,
                                                         request->target_frame,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_tool_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Move Tool to Frame");

  auto [success, joint_names, joint_values] = move_group_to_frame("PM_Robot_Tool_TCP",
                                                         tool_move_group,
                                                         request->endeffector_frame_override,
                                                         request->target_frame,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}

void move_laser_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                     std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_frame("PM_Robot_Laser_TCP",
                                                         laser_move_group,
                                                         request->endeffector_frame_override,
                                                         request->target_frame,
                                                         request->translation,
                                                         request->rotation,
                                                         request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;

  return;
}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // create a move_to_frame request
  auto move_to_frame_request = std::make_shared<pm_moveit_interfaces::srv::MoveToFrame::Request>();
  default_endeffector_string = move_to_frame_request->endeffector_frame_override;

  auto const pm_moveit_server_node = std::make_shared<rclcpp::Node>(
      "pm_moveit_server",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto callback_group_re = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions node_options_1;
  node_options_1.callback_group = callback_group_re;

  auto callback_group_me = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions node_options_2;
  node_options_2.callback_group = callback_group_me;

  rclcpp::ExecutorOptions exec_options;
  //exec_options->num_threads = 4;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pm_moveit_server_node);
  
  auto const logger = rclcpp::get_logger("hello_moveit");

  laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
  tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
  Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
  plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
  //auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  //psm->startSceneMonitor("/move_group/monitored_planning_scene");
  //auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, laser_move_group->getRobotModel());
  //auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", "moveit_cpp_tutorial", laser_move_group->getRobotModel());

  laser_grp_visual_tools =  std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
  laser_grp_visual_tools->deleteAllMarkers();
  laser_grp_visual_tools->loadRemoteControl();
  //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
  //auto test = moveit_visual_tools::MoveItVisualTools::MoveItVisualTools("/world", rviz_visual_tools::RVIZ_MARKER_TOPIC);
  //auto test = moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node);
  //laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(psm);

  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node, "robot_description");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(pm_moveit_server_node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Dont know why this has to be called at this point, but otherwith the service callbacks get stuck.
  //auto current_state_cam1 = Cam1_move_group->getCurrentState(1.0);
  //auto current_state_tool = tool_move_group->getCurrentState(1.0);
  //auto current_state_laser = laser_move_group->getCurrentState(1.0);

  rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&execute_plan,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", std::bind(&move_cam_one_to_pose,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", std::bind(&move_group_tool,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", std::bind(&move_group_laser,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_cam1_to_frame", std::bind(&move_cam_one_to_frame,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_tool_to_frame", std::bind(&move_tool_to_frame,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_laser_to_frame", std::bind(&move_laser_to_frame,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_cam1_relative", std::bind(&move_cam1_relative,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_tool_relative", std::bind(&move_tool_relative,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_laser_relative", std::bind(&move_laser_relative,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_cam1_to_pose", std::bind(&move_cam_one_to_pose,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_tool_to_pose", std::bind(&move_tool_to_pose,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_laser_to_pose", std::bind(&move_laser_to_pose,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

  xyz_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_xyz_axis_controller/joint_trajectory", 10);
  t_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_t_axis_controller/joint_trajectory", 10);
  auto joint_state_subscriber = pm_moveit_server_node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStateCallback, node_options_1);
  
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Ready for operation...");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}








