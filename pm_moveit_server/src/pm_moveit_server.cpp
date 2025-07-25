#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuple>
#include <future>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include "pm_moveit_interfaces/srv/get_gonio_solution.hpp"
#include "pm_moveit_interfaces/srv/move_relative.hpp"
#include "pm_moveit_interfaces/srv/move_to_pose.hpp"
#include "pm_moveit_interfaces/srv/move_to_frame.hpp"
#include "pm_moveit_interfaces/srv/align_gonio.hpp"
#include "pm_msgs/srv/empty_with_success.hpp"
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <iomanip>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

Eigen::Affine3d poseMsgToAffine(const geometry_msgs::msg::Pose &pose_msg)
{
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

geometry_msgs::msg::Pose round_pose(geometry_msgs::msg::Pose pose_to_round)
{
  geometry_msgs::msg::Pose rounded_pose;
  int decimal_precision = 3;

  const double multiplier = std::pow(10, decimal_precision);

  rounded_pose.orientation.x = std::round(pose_to_round.orientation.x * multiplier) / multiplier;
  rounded_pose.orientation.y = std::round(pose_to_round.orientation.y * multiplier) / multiplier;
  rounded_pose.orientation.z = std::round(pose_to_round.orientation.z * multiplier) / multiplier;
  rounded_pose.orientation.w = std::round(pose_to_round.orientation.w * multiplier) / multiplier;
  rounded_pose.position = pose_to_round.position;
  return rounded_pose;
}

geometry_msgs::msg::Quaternion quaternion_multiply(geometry_msgs::msg::Quaternion q0, geometry_msgs::msg::Quaternion q1)
{
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

// Global Variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> laser_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Cam1_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> tool_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> dispenser_1k_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gonio_left_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gonio_right_move_group;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> confocal_head_move_group;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> smarpod_move_group;

std::shared_ptr<rclcpp::Node> pm_moveit_server_node;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> laser_grp_visual_tools;

std::shared_ptr<rclcpp::Client<pm_moveit_interfaces::srv::GetGonioSolution>> get_gonio_right_solution_client;
std::shared_ptr<rclcpp::Client<pm_moveit_interfaces::srv::GetGonioSolution>> get_gonio_left_solution_client;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
sensor_msgs::msg::JointState::SharedPtr global_joint_state;
// create shared pointer to node publisher
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> xyz_trajectory_publisher;
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> t_trajectory_publisher;
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> smarpod_trajectory_publisher;
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> gonio_right_trajectory_publisher;
std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> gonio_left_trajectory_publisher;

std::string default_endeffector_string;
std::string global_return_massage;
std::chrono::high_resolution_clock::time_point init_time;
std::chrono::high_resolution_clock::time_point end_time;
std::chrono::high_resolution_clock::time_point start_ik_solution_time;
std::chrono::high_resolution_clock::time_point end_ik_solution_time;
std::chrono::high_resolution_clock::time_point start_plan_time;
std::chrono::high_resolution_clock::time_point end_plan_time;
std::chrono::high_resolution_clock::time_point start_execute_time;
std::chrono::high_resolution_clock::time_point end_execute_time;
std::chrono::high_resolution_clock::time_point start_wait_for_movement_end;
std::chrono::high_resolution_clock::time_point end_wait_for_movement_end;
std::chrono::high_resolution_clock::time_point start_init_request;
std::chrono::high_resolution_clock::time_point end_init_request;

bool use_trajectory_planning;

void log_pose(std::string pose_text, geometry_msgs::msg::Pose pose)
{
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), pose_text.c_str());
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position X %.9f", pose.position.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position Y %.9f", pose.position.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Position Z %.9f", pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation W %f", pose.orientation.w);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation X %f", pose.orientation.x);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation Y %f", pose.orientation.y);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Orientation Z %f", pose.orientation.z);
}

void log_time_measures()
{
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Time Measures:");
  // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Init Time: %f", init_time.count());
  std::chrono::duration<double> init_request_time = end_init_request - start_init_request;
  std::chrono::duration<double> ik_solution_time = end_ik_solution_time - start_ik_solution_time;
  std::chrono::duration<double> plan_time = end_plan_time - start_plan_time;
  std::chrono::duration<double> execute_time = end_execute_time - start_execute_time;
  std::chrono::duration<double> wait_time = end_wait_for_movement_end - start_wait_for_movement_end;
  std::chrono::duration<double> total_time = end_time - init_time;

  double total_seconds = total_time.count();

  double init_request_pct = (init_request_time.count() / total_seconds) * 100.0;
  double ik_solution_pct = (ik_solution_time.count() / total_seconds) * 100.0;
  double plan_pct = (plan_time.count() / total_seconds) * 100.0;
  double execute_pct = (execute_time.count() / total_seconds) * 100.0;
  double wait_pct = (wait_time.count() / total_seconds) * 100.0;
  double pct_missing = 100.0 - (init_request_pct + ik_solution_pct + plan_pct + execute_pct + wait_pct);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Init Request Time: %.6f s (%.2f%%)", init_request_time.count(), init_request_pct);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "IK Solution Time: %.6f s (%.2f%%)", ik_solution_time.count(), ik_solution_pct);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Plan Time: %.6f s (%.2f%%)", plan_time.count(), plan_pct);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Execute Time: %.6f s (%.2f%%)", execute_time.count(), execute_pct);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Wait for Movement Time: %.6f s (%.2f%%)", wait_time.count(), wait_pct);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Total Time: %.6f s (100.00%%)", total_seconds);
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Missing Time Percentage: %.2f%%", pct_missing);
}

bool execute_plan(const std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Request> request,
                  std::shared_ptr<pm_moveit_interfaces::srv::ExecutePlan::Response> response)
{
  if (request->run)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Yes");
    response->success = true;
  }

  return true;
}

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Process the received joint state message
  // For example, print the names and positions of the joints
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    // std::cout << "Joint Name: " << msg->name[i] << ", Position: " << msg->position[i] << std::endl;
    //  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Joint Name: %s, Value: %f",msg->name[i].c_str(), msg->position[i]);
  }
  global_joint_state = msg;
}

bool check_goal_reached(std::vector<std::string> target_joints, std::vector<double> target_joint_values, float delta_trans, float delta_rot)
{
  float delta_value;
  for (size_t i = 0; i < target_joints.size(); i++)
  {
    if (target_joints[i] == "T_Axis_Joint" ||
        target_joints[i] == "SP_A_Joint" ||
        target_joints[i] == "SP_B_Joint" ||
        target_joints[i] == "SP_C_Joint" ||
        target_joints[i] == "Gonio_Right_Stage_1_Joint" ||
        target_joints[i] == "Gonio_Right_Stage_2_Joint" ||
        target_joints[i] == "Gonio_Left_Stage_1_Joint" ||
        target_joints[i] == "Gonio_Left_Stage_2_Joint")
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
    if (it == global_joint_state->name.end())
    {
      // Joint not found
      // Handle error or return false
      RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Joint not found in joint state!");
      return false;
    }
    int current_joint_index = std::distance(global_joint_state->name.begin(), it);
    float current_joint_value = global_joint_state->position[current_joint_index];
    float differrence = std::abs(current_joint_value - target_joint_values[i]);

    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint: %s, Target: %.9f, Current: %.9f, Delta: %.9f, Delta_Velue: %.9f", target_joints[i].c_str(), target_joint_values[i], current_joint_value, differrence, delta_value);

    if (differrence > delta_value)
    {
      return false;
    }
  }
  return true;
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> calculate_IK(std::string planning_group,
                                                                             std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                             geometry_msgs::msg::Pose target_pose)
{
  start_ik_solution_time = std::chrono::high_resolution_clock::now();

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

  // get joint limits
  for (size_t i = 0; i < joint_bounds.size(); i++)
  {
    min_joint_values.push_back(joint_bounds[i][0][0].min_position_);
    max_joint_values.push_back(joint_bounds[i][0][0].max_position_);

    RCLCPP_DEBUG(rclcpp::get_logger("pm_moveit"), " Lower: %.9f, Upper: %.9f", joint_bounds[i][0][0].min_position_, joint_bounds[i][0][0].max_position_);
    // RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), " Lower: %.9f, Upper: %.9f", joint_bounds[i][0][0].min_position_, joint_bounds[i][0][0].max_position_);
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
    tf2::convert(end_effector_state, tf2Transform);
    // tf2::Vector3 endeffector_pose_planed = tf2Transform.getOrigin();
    //  This is the same as the calculated Endeffector Pose
    // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose X %f", endeffector_pose_planed.getX());
    // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Y %f", endeffector_pose_planed.getY());
    // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Planned Endeffector Pose Z %f", endeffector_pose_planed.getZ());
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
  end_ik_solution_time = std::chrono::high_resolution_clock::now();
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

bool check_frame_is_in_chain(std::string target_frame, std::string parent_frame)
{
  // Check if the target frame is in the chain of the parent frame

  try
  {
    std::string yaml_string = tf_buffer_->allFramesAsYAML();
    YAML::Node yaml_node = YAML::Load(yaml_string);
    std::string current_frame = target_frame;
    int iterator = 0;
    while (current_frame != "world")
    {
      if (current_frame == parent_frame)
      {
        return true;
      }
      if (iterator > 100)
      {
        return false;
      }
      // RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Current Frame: %s", current_frame.c_str());

      for (YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it)
      {
        std::string frame_id = it->first.as<std::string>();
        YAML::Node frame_data = it->second;

        if (frame_id == current_frame)
        {
          current_frame = frame_data["parent"].as<std::string>();
          // RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Frame ID: %s", frame_id.c_str());
          // RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Parent: %s", frame_data["parent"].as<std::string>().c_str());
          if (current_frame == parent_frame)
          {
            return true;
          }
        }
        else
        {
          continue;
        }
      }

      iterator++;
    }
  }
  catch (const YAML::Exception &ex)
  {
    throw std::runtime_error("YAML exception: " + std::string(ex.what()));
  }
  catch (const std::exception &ex)
  {
    throw std::runtime_error("Error finding parent frame: " + std::string(ex.what()));
  }
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
  // geometry_msgs::msg::TransformStamped frame_world_transform;
  // tf2::fromMsg(pose, frame_world_transform);
  // geometry_msgs::msg::TransformStamped rel_transform;
  // rel_transform.transform.translation = translation;
  // rel_transform.transform.rotation = rotation;
  // rel_transform.transform.rotation = quaternion_multiply(pose.orientation, rotation);
  tf2::Transform transform_1;
  transform_1.setOrigin(tf2::Vector3(pose.position.x,
                                     pose.position.y,
                                     pose.position.z));
  transform_1.setRotation(tf2::Quaternion(pose.orientation.x,
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
  // tf2::fromMsg(transform_res, pose);
  return pose;
}

void log_transform_stamped(std::string transform_text, geometry_msgs::msg::TransformStamped transform)
{
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), transform_text.c_str());
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Transform from %s to %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Position X %.9f", transform.transform.translation.x);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Position Y %.9f", transform.transform.translation.y);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Position Z %.9f", transform.transform.translation.z);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Orientation W %f", transform.transform.rotation.w);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Orientation X %f", transform.transform.rotation.x);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Orientation Y %f", transform.transform.rotation.y);
  RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Orientation Z %f", transform.transform.rotation.z);
}

geometry_msgs::msg::Pose get_pose_endeffector_override(std::string initial_endeffector_frame,
                                                       std::string endeffector_override_frame,
                                                       geometry_msgs::msg::Pose target_pose)
{
  bool success_frame;
  geometry_msgs::msg::Pose pose_rel;
  geometry_msgs::msg::TransformStamped rel_transform;
  geometry_msgs::msg::TransformStamped rel_transform2;
  std::tie(success_frame, rel_transform) = get_pose_of_frame_in_frame(endeffector_override_frame, initial_endeffector_frame);
  // std::tie(success_frame, rel_transform) = get_pose_of_frame_in_frame(initial_endeffector_frame, endeffector_override_frame);
  std::tie(success_frame, rel_transform2) = get_pose_of_frame_in_frame(initial_endeffector_frame, endeffector_override_frame);

  // // geometry_msgs::msg::TransformStamped world_endeffector_override;
  // // geometry_msgs::msg::TransformStamped world_initial_endeffector;
  // geometry_msgs::msg::TransformStamped transformmm_3;
  // geometry_msgs::msg::TransformStamped transformmm_4;

  // std::tie(success_frame, world_endeffector_override) = get_pose_of_frame_in_frame(endeffector_override_frame, "world");
  //  std::tie(success_frame, world_endeffector_override) = get_pose_of_frame_in_frame( "world", endeffector_override_frame);

  // std::tie(success_frame, world_initial_endeffector) = get_pose_of_frame_in_frame( "world", initial_endeffector_frame);

  // std::tie(success_frame, transformmm_3) = get_pose_of_frame_in_frame( endeffector_override_frame, initial_endeffector_frame);

  // std::tie(success_frame, transformmm_4) = get_pose_of_frame_in_frame(initial_endeffector_frame, endeffector_override_frame);

  // log_transform_stamped("This is important!!!!", rel_transform);

  // log_transform_stamped("Transform 1", world_endeffector_override);
  // log_transform_stamped("Transform 2", world_initial_endeffector);
  // log_transform_stamped("Transform 3", transformmm_3);
  // log_transform_stamped("Transform 4", transformmm_4);

  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // // log rel transform
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Rel Transform between %s and %s", initial_endeffector_frame.c_str(), endeffector_override_frame.c_str());
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "X: %f", rel_transform2.transform.translation.x);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Y: %f", rel_transform2.transform.translation.y);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Z: %f", rel_transform2.transform.translation.z);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Q_X: %f", rel_transform2.transform.rotation.x);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Q_Y: %f", rel_transform2.transform.rotation.y);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Q_Z: %f", rel_transform2.transform.rotation.z);
  // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Q_W: %f", rel_transform2.transform.rotation.w);

  if (!success_frame)
  {
    return target_pose;
  }

  tf2::Transform target_pose_transform;
  target_pose_transform.setOrigin(tf2::Vector3(target_pose.position.x,
                                               target_pose.position.y,
                                               target_pose.position.z));

  target_pose_transform.setRotation(tf2::Quaternion(target_pose.orientation.x,
                                                    target_pose.orientation.y,
                                                    target_pose.orientation.z,
                                                    target_pose.orientation.w));

  tf2::Transform transform_2;
  tf2::fromMsg(rel_transform.transform, transform_2);

  // tf2::Transform transform_3;
  // tf2::fromMsg(world_endeffector_override.transform, transform_3);

  // tf2::Transform transform_4;
  // tf2::fromMsg(rel_transform2.transform, transform_4);

  geometry_msgs::msg::Pose pose;
  tf2::Transform transform_res;

  // I dont know why this does not work with the 1K_dispencer_TCP
  // this is now the workaround, I just can not figure out what the issue is, but it seems to work like this, although i dont know why...
  if (endeffector_override_frame == "1K_Dispenser_TCP")
  {
    transform_res.mult(transform_2, target_pose_transform);
  }
  else if (initial_endeffector_frame == "Cam1_Toolhead_TCP")
  {
    // RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "USING DIFFERENT RULE");
    transform_res.setOrigin(tf2::Vector3(target_pose.position.x - rel_transform2.transform.translation.x,
                                         target_pose.position.y - rel_transform2.transform.translation.y,
                                         target_pose.position.z - rel_transform2.transform.translation.z));
  }
  else
  {
    transform_res.mult(target_pose_transform, transform_2);
  }
  // transform_res.mult(transform_1, transform_2);
  // transform_res.mult(transform_2, transform_1);
  //  auto vector = transform_res.getOrigin();
  //  pose.position.x = vector.x();
  //  pose.position.y = vector.y();
  //  pose.position.z = vector.z();
  //  auto quat = transform_res.getRotation();
  //  pose.orientation.x = quat.x();
  //  pose.orientation.y = quat.y();
  //  pose.orientation.z = quat.z();
  //  pose.orientation.w = quat.w();
  tf2::toMsg(transform_res, pose);
  // tf2::fromMsg(transform_res, pose);
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

std::tuple<bool, std::string> set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                             std::vector<double> target_joint_values,
                                             bool execute_movement)
{

  start_plan_time = std::chrono::high_resolution_clock::now();

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
    return std::make_tuple(false, "Planing failed!");
  }
  end_plan_time = std::chrono::high_resolution_clock::now();

  // laser_grp_visual_tools->deleteAllMarkers();
  // auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
  // laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  // laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
  // laser_grp_visual_tools->prompt("next step");
  // laser_grp_visual_tools->trigger();

  start_execute_time = std::chrono::high_resolution_clock::now();
  // Execute the plan
  if (success_calculate_plan && execute_movement)
  {
    move_group->execute(*plan);
    end_execute_time = std::chrono::high_resolution_clock::now();
    return std::make_tuple(true, "Planing successfull!");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan calculated successfull, but not execution was not demanded!");
    end_execute_time = std::chrono::high_resolution_clock::now();
    return std::make_tuple(true, "Plan calculated successfull, but the plan was not executed because 'execute_movement' was not set to true!");
  }
}

std::tuple<bool, std::string> set_move_group_orientation(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                         double x,
                                                         double y,
                                                         double z,
                                                         double w,
                                                         bool execute_movement)
{
  bool success_calculate_plan = false;
  move_group->setPlanningTime(10);
  move_group->setStartStateToCurrentState();

  move_group->setGoalJointTolerance(1e-9);

  move_group->setOrientationTarget(x, y, z, w);
  move_group->setNumPlanningAttempts(100);
  move_group->setReplanAttempts(10000);
  success_calculate_plan = (move_group->plan(*plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_calculate_plan)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Planing failed!");
    return std::make_tuple(false, "Planing failed!");
  }

  // laser_grp_visual_tools->deleteAllMarkers();
  // auto jmg = move_group->getRobotModel()->getJointModelGroup(planning_group);
  // laser_grp_visual_tools->publishText(target_pose,"Test",rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
  // laser_grp_visual_tools->publishTrajectoryLine(plan->trajectory_, jmg, rviz_visual_tools::LIME_GREEN);
  // laser_grp_visual_tools->prompt("next step");
  // laser_grp_visual_tools->trigger();

  // Execute the plan
  if (success_calculate_plan && execute_movement)
  {
    move_group->execute(*plan);
    return std::make_tuple(true, "Planing successfull!");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Plan calculated successfull, but not execution was not demanded!");
    return std::make_tuple(true, "Plan calculated successfull, but the plan was not executed because 'execute_movement' was not set to true!");
  }
}

void publish_target_joint_trajectory_xyzt(std::string planning_group,
                                          std::vector<double> target_joint_values,
                                          float time_from_start)
{
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->joint_names = {"X_Axis_Joint", "Y_Axis_Joint", "Z_Axis_Joint"}; // Specify joint names

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2]}; // Specify joint positions
  point.velocities = {0.0, 0.0, 0.0};                                                         // Specify joint velocities
  point.accelerations = {0.0, 0.0, 0.0};                                                      // Specify joint accelerations
  point.time_from_start.sec = time_from_start;                                                // Specify duration
  trajectory_msg->points.push_back(point);
  xyz_trajectory_publisher->publish(*trajectory_msg);

  // If planning group is PM_Robot_Tool_TCP, publish T_Axis_Joint trajectory
  if (planning_group == "PM_Robot_Tool_TCP")
  {
    auto t_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    t_trajectory_msg->joint_names = {"T_Axis_Joint"}; // Specify joint names
    trajectory_msgs::msg::JointTrajectoryPoint t_point;
    t_point.positions = {target_joint_values[3]};  // Specify joint positions
    t_point.velocities = {0.0};                    // Specify joint velocities
    t_point.accelerations = {0.0};                 // Specify joint accelerations
    t_point.time_from_start.sec = time_from_start; // Specify duration
    t_trajectory_msg->points.push_back(t_point);
    t_trajectory_publisher->publish(*t_trajectory_msg);
  }
}

void publish_target_joint_trajectory_gonio_left(std::vector<double> target_joint_values,
                                                float time_from_start)
{
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->joint_names = {"Gonio_Left_Stage_1_Joint", "Gonio_Left_Stage_2_Joint"}; // Specify joint names

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_joint_values[0], target_joint_values[1]}; // Specify joint positions
  point.velocities = {0.0, 0.0};                                      // Specify joint velocities
  point.accelerations = {0.0, 0.0};                                   // Specify joint accelerations
  point.time_from_start.sec = time_from_start;                        // Specify duration
  trajectory_msg->points.push_back(point);
  gonio_left_trajectory_publisher->publish(*trajectory_msg);
  return;
}

void publish_target_joint_trajectory_gonio_right(std::vector<double> target_joint_values,
                                                 float time_from_start)
{
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->joint_names = {"Gonio_Right_Stage_1_Joint", "Gonio_Right_Stage_2_Joint"}; // Specify joint names

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_joint_values[0], target_joint_values[1]}; // Specify joint positions
  point.velocities = {0.0, 0.0};                                      // Specify joint velocities
  point.accelerations = {0.0, 0.0};                                   // Specify joint accelerations
  point.time_from_start.sec = time_from_start;                        // Specify duration
  trajectory_msg->points.push_back(point);
  gonio_right_trajectory_publisher->publish(*trajectory_msg);
  return;
}

void publish_target_joint_trajectory_smarpod(std::string planning_group,
                                             std::vector<double> target_joint_values,
                                             float time_from_start)
{
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->joint_names = {"SP_X_Joint", "SP_Y_Joint", "SP_Z_Joint", "SP_A_Joint", "SP_B_Joint", "SP_C_Joint"}; // Specify joint names

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_joint_values[0], target_joint_values[1], target_joint_values[2], target_joint_values[3], target_joint_values[4], target_joint_values[5]}; // Specify joint positions
  point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                                                                                                                  // Specify joint velocities
  point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                                                                                                               // Specify joint accelerations
  point.time_from_start.sec = time_from_start;                                                                                                                        // Specify duration
  trajectory_msg->points.push_back(point);
  smarpod_trajectory_publisher->publish(*trajectory_msg);
}

void wait_for_movement_to_finish(std::vector<std::string> joint_names, std::vector<double> target_joint_values, float lateral_tolerance, float angular_tolerance)
{
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for goal to be reached...");
  int max_wait_time_counter = 50;
  int wait_time_counter = 0;

  while (check_goal_reached(joint_names, target_joint_values, lateral_tolerance, angular_tolerance) == false && wait_time_counter < max_wait_time_counter)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wait_time_counter++;
  }
  if (wait_time_counter >= max_wait_time_counter)
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "WARNING: Goal not reached in time! Assuming goal reached!");
  }
  // else
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Goal reached!");
  // }
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

void set_initial_state_move_groups()
{
  laser_move_group->setStartStateToCurrentState();
  tool_move_group->setStartStateToCurrentState();
  Cam1_move_group->setStartStateToCurrentState();
  dispenser_1k_move_group->setStartStateToCurrentState();
  gonio_left_move_group->setStartStateToCurrentState();
  gonio_right_move_group->setStartStateToCurrentState();
  smarpod_move_group->setStartStateToCurrentState();
}

std::tuple<bool, std::vector<std::string>, std::vector<double>, std::string> move_group_relative(std::string planning_group,
                                                                                                 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                                                 geometry_msgs::msg::Vector3 translation,
                                                                                                 geometry_msgs::msg::Quaternion rotation,
                                                                                                 bool execute_movement)
{
  init_time = std::chrono::high_resolution_clock::now();

  // START Init Request
  start_init_request = std::chrono::high_resolution_clock::now();

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  std::string msg;

  move_group->setStartStateToCurrentState();

  bool success_ik;
  // bool extract_frame_success;
  //  Get the target_pose
  auto [extract_frame_success, target_pose] = get_pose_of_frame(endeffector);

  // This should normaly not happen, because the searched frame is the endeffector, which must exist
  if (!extract_frame_success)
  {
    // If frame is not found, return false
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Specified frame not found!");
    msg = "Specified frame not found!";
    return std::make_tuple(false, joint_names, target_joint_values, msg);
  }

  // Check if rotation is valid and set to default if not
  rotation = check_rotation(rotation);

  target_pose = add_translation_rotation_to_pose(target_pose, translation, rotation);
  log_pose("Calculated Endeffector Pose: ", target_pose);
  end_init_request = std::chrono::high_resolution_clock::now();

  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  // START Plan&Execute
  if (!success_ik)
  {
    msg = "IK solution not found!";
    return {false, joint_names, target_joint_values, msg};
  }

  bool move_success = false;

  if (use_trajectory_planning)
  {
    auto [move_suc, msg] = set_move_group(move_group, target_joint_values, execute_movement);

    if (!move_suc || !execute_movement)
    {
      return {move_suc, joint_names, target_joint_values, msg};
    }
    move_success = true;
  }
  else
  {
    start_plan_time = std::chrono::high_resolution_clock::now();
    end_plan_time = std::chrono::high_resolution_clock::now();
    start_execute_time = std::chrono::high_resolution_clock::now();
    if (planning_group == "smarpod_endeffector")
    {
      publish_target_joint_trajectory_smarpod(planning_group, target_joint_values, 0.0);
    }
    else
    {
      publish_target_joint_trajectory_xyzt(planning_group, target_joint_values, 0.0);
    }
    end_execute_time = std::chrono::high_resolution_clock::now();
    move_success = true;
  }

  // END Plan&Execute

  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;

  start_wait_for_movement_end = std::chrono::high_resolution_clock::now();

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  if (planning_group == "smarpod_endeffector")
  {
    publish_target_joint_trajectory_smarpod(planning_group, target_joint_values, 0.1);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }

  else
  {
    publish_target_joint_trajectory_xyzt(planning_group, target_joint_values, 0.1);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }
  end_wait_for_movement_end = std::chrono::high_resolution_clock::now();

  log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");
  msg = "Movement successfull!";
  end_time = std::chrono::high_resolution_clock::now();

  log_time_measures();

  return std::make_tuple(move_success, joint_names, target_joint_values, msg);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> move_group_to_frame(std::string planning_group,
                                                                                    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                                    std::string endeffector_frame_override,
                                                                                    std::string target_frame,
                                                                                    geometry_msgs::msg::Vector3 translation,
                                                                                    geometry_msgs::msg::Quaternion rotation,
                                                                                    bool execute_movement)
{
  init_time = std::chrono::high_resolution_clock::now();

  // START Init Request
  start_init_request = std::chrono::high_resolution_clock::now();

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  // bool extract_frame_success;
  //  Get the target_pose
  auto [extract_frame_success, target_pose] = get_pose_of_frame(target_frame);

  if (((planning_group == "PM_Robot_Tool_TCP") or (planning_group == "PM_Robot_Cam1_TCP")) and (endeffector_frame_override != default_endeffector_string))
  {

    bool valid_endeffector = check_frame_is_in_chain(endeffector_frame_override, "Z_Axis");
    if (!valid_endeffector)
    {
      RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "For MoveGroup 'PM_Robot_Tool_TCP' endeffector_override %s frame not in chain of Z_Axis!", endeffector_frame_override.c_str());
      return std::make_tuple(false, joint_names, target_joint_values);
    }
  }

  auto origin_pose = target_pose;
  // lo
  //  This should normaly not happen, because the searched frame is the endeffector, which must exist
  if (!extract_frame_success)
  {
    // If frame is not found, return false
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Specified frame not found!");
    return std::make_tuple(false, joint_names, target_joint_values);
  }

  log_pose("Target frame pose: ", target_pose);

  if (endeffector_frame_override != default_endeffector_string)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Overriding Endeffector '%s' Frame to: '%s'", endeffector.c_str(), endeffector_frame_override.c_str());
    target_pose = get_pose_endeffector_override(endeffector, endeffector_frame_override, target_pose);
  }

  log_pose("Before translation", target_pose);

  auto diff_pose = geometry_msgs::msg::Pose();

  diff_pose.position.x = origin_pose.position.x - target_pose.position.x;
  diff_pose.position.y = origin_pose.position.y - target_pose.position.y;
  diff_pose.position.z = origin_pose.position.z - target_pose.position.z;

  log_pose("Diff Pose: ", diff_pose);

  // !!!!!!!! Here the rotation is rounded to account for small deviations in the pose !!!!
  // This should later be deleted!!!
  target_pose = round_pose(target_pose);

  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "BE AWARE! Rotations are rounded to account for small deviations in the pose!! This might cause errors in the orientation.");

  // Check if rotation is valid and set to default if not
  rotation = check_rotation(rotation);

  target_pose = add_translation_rotation_to_pose(target_pose, translation, rotation);
  log_pose("Calculated target endeffector pose: ", target_pose);

  end_init_request = std::chrono::high_resolution_clock::now();

  // END Init Request

  // START Plan IK
  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  // END Plan IK

  // START Plan&Execute
  if (!success_ik)
  {
    return {false, joint_names, target_joint_values};
  }

  bool move_success = false;
  std::string res_msg;

  if (use_trajectory_planning)
  {
    auto [move_suc, msg] = set_move_group(move_group, target_joint_values, execute_movement);

    if (!move_suc || !execute_movement)
    {
      return {move_suc, joint_names, target_joint_values};
    }
    move_success = true;
  }
  else
  {
    start_plan_time = std::chrono::high_resolution_clock::now();
    end_plan_time = std::chrono::high_resolution_clock::now();
    start_execute_time = std::chrono::high_resolution_clock::now();
    if (planning_group == "smarpod_endeffector")
    {
      publish_target_joint_trajectory_smarpod(planning_group, target_joint_values, 0.0);
    }
    else
    {
      publish_target_joint_trajectory_xyzt(planning_group, target_joint_values, 0.0);
    }
    end_execute_time = std::chrono::high_resolution_clock::now();
    move_success = true;
  }

  // END Plan&Execute

  // START Wait For Movement
  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;

  start_wait_for_movement_end = std::chrono::high_resolution_clock::now();
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  if (planning_group == "smarpod_endeffector")
  {
    publish_target_joint_trajectory_smarpod(planning_group, target_joint_values, 0.0);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }
  else
  {
    publish_target_joint_trajectory_xyzt(planning_group, target_joint_values, 0.0);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }
  end_wait_for_movement_end = std::chrono::high_resolution_clock::now();
  // END Wait for Movement

  // This takes 250 ms because it waits...
  // log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");

  end_time = std::chrono::high_resolution_clock::now();

  log_time_measures();
  return std::make_tuple(move_success, joint_names, target_joint_values);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> move_group_to_pose(std::string planning_group,
                                                                                   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                                   geometry_msgs::msg::Pose target_pose,
                                                                                   std::string endeffector_frame_override,
                                                                                   bool execute_movement)
{
  init_time = std::chrono::high_resolution_clock::now();

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  // Get the target_pose

  if (endeffector_frame_override != default_endeffector_string)
  {
    RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Overriding Endeffector Frame to: %s", endeffector_frame_override.c_str());
    target_pose = get_pose_endeffector_override(endeffector, endeffector_frame_override, target_pose);
  }

  log_pose("Calculated Endeffector Pose: ", target_pose);

  std::tie(success_ik, joint_names, target_joint_values) = calculate_IK(planning_group, move_group, target_pose);

  if (!success_ik)
  {
    return {false, joint_names, target_joint_values};
  }
  auto [move_success, msg] = set_move_group(move_group, target_joint_values, execute_movement);

  if (!move_success || !execute_movement)
  {
    return {move_success, joint_names, target_joint_values};
  }

  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  log_target_pose_delta(endeffector, target_pose);

  // this may not be necessary anymore
  if (planning_group == "smarpod_endeffector")
  {
    publish_target_joint_trajectory_smarpod(planning_group, target_joint_values, 0.1);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }

  else
  {
    publish_target_joint_trajectory_xyzt(planning_group, target_joint_values, 0.1);
    wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  }

  log_target_pose_delta(endeffector, target_pose);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");

  return std::make_tuple(move_success, joint_names, target_joint_values);
}

std::tuple<bool, std::vector<std::string>, std::vector<double>> align_gonio(std::string planning_group,
                                                                            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                                                            std::string endeffector_frame_override,
                                                                            std::string target_frame,
                                                                            geometry_msgs::msg::Vector3 rotation_offset_deg,
                                                                            bool execute_movement)
{
  init_time = std::chrono::high_resolution_clock::now();
  // START Init Request
  start_init_request = std::chrono::high_resolution_clock::now();

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Aligning Request Received Gonio...");

  std::string endeffector = move_group->getEndEffectorLink();
  geometry_msgs::msg::Quaternion target_rotation;
  std::vector<double> target_joint_values;
  std::vector<std::string> joint_names;
  bool success_ik;
  std::string target_endeffector_frame;
  std::string endeffector_frame_parent;
  // Get the pose of the gonio
  bool gonio_pose_success = false;
  geometry_msgs::msg::Pose gonio_pose;

  auto request = std::make_shared<pm_moveit_interfaces::srv::GetGonioSolution::Request>();
  auto response = std::make_shared<pm_moveit_interfaces::srv::GetGonioSolution::Response>();

  if (endeffector_frame_override == "use_default_frame")
  {
    target_endeffector_frame = endeffector;
  }
  else
  {
    target_endeffector_frame = endeffector_frame_override;
  }

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Target Endeffector Frame: %s", target_endeffector_frame.c_str());

  if (planning_group == "PM_Robot_Gonio_Right")
  {
    endeffector_frame_parent = "Gonio_Base_Right";
  }
  else if (planning_group == "PM_Robot_Gonio_Left")
  {
    endeffector_frame_parent = "Gonio_Left_Base";
  }

  bool endeffector_frame_valid = check_frame_is_in_chain(target_endeffector_frame, endeffector_frame_parent);

  if (!endeffector_frame_valid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Endeffector frame not valid!");
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Endeffector frame: %s", target_endeffector_frame.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Endeffector frame parent: %s", endeffector_frame_parent.c_str());
    return {false, joint_names, target_joint_values};
  }

  bool target_frame_valid = check_frame_is_in_chain(target_frame, "Z_Axis");

  if (!target_frame_valid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Target frame not valid!");
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Target frame: %s", target_frame.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("pm_moveit"), "Target frame parent: Z_Axis");
    return {false, joint_names, target_joint_values};
  }

  end_init_request = std::chrono::high_resolution_clock::now();

  request->gripper_endeffector = target_frame;
  request->gonio_endeffector = target_endeffector_frame;

  start_plan_time = std::chrono::high_resolution_clock::now();
  if (planning_group == "PM_Robot_Gonio_Right")
  {
    auto future = get_gonio_right_solution_client->async_send_request(request);
    response = future.get();
  }
  else if (planning_group == "PM_Robot_Gonio_Left")
  {
    auto future = get_gonio_left_solution_client->async_send_request(request);
    response = future.get();
  }

  // auto response = future.get();
  // log response
  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Response: %s", response->success ? "true" : "false");
  // log joint values
  RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint Values: ");
  for (size_t i = 0; i < response->joint_values.size(); i++)
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Joint %s: %f", response->joint_names[i].c_str(), response->joint_values[i]);
  }
  end_plan_time = std::chrono::high_resolution_clock::now();

  // bool move_success = false;
  target_joint_values.push_back(response->joint_values[1]);
  target_joint_values.push_back(response->joint_values[2]);
  start_execute_time = std::chrono::high_resolution_clock::now();
  auto [move_success, move_msg] = set_move_group(move_group, target_joint_values, execute_movement);

  if (!move_success || !execute_movement)
  {
    return {move_success, joint_names, target_joint_values};
  }
  end_execute_time = std::chrono::high_resolution_clock::now();

  start_wait_for_movement_end = std::chrono::high_resolution_clock::now();
  float lateral_tolerance_coarse = 1e-2;
  float angular_tolerance_coarse = 0.01;
  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_coarse, angular_tolerance_coarse);

  // log_target_pose_delta(endeffector, target_pose);

  // // this may not be necessary anymore

  if (planning_group == "PM_Robot_Gonio_Right")
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Using Gonio Right");

    publish_target_joint_trajectory_gonio_right(target_joint_values, 1.0);
  }
  else if (planning_group == "PM_Robot_Gonio_Left")
  {
    RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "Using Gonio Left");
    publish_target_joint_trajectory_gonio_left(target_joint_values, 1.0);
  }

  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);
  // log_target_pose_delta(endeffector, target_pose);
  end_wait_for_movement_end = std::chrono::high_resolution_clock::now();
  end_time = std::chrono::high_resolution_clock::now();
  log_time_measures();

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Waiting for next command...");

  return std::make_tuple(move_success, joint_names, target_joint_values);
}

// RELATIVE MOVEMENT
void move_cam1_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                        std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values, msg] = move_group_relative("PM_Robot_Cam1_TCP",
                                                                       Cam1_move_group,
                                                                       request->translation,
                                                                       request->rotation,
                                                                       request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;
  response->message = msg;

  return;
}

void move_tool_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                        std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values, msg] = move_group_relative("PM_Robot_Tool_TCP",
                                                                       tool_move_group,
                                                                       request->translation,
                                                                       request->rotation,
                                                                       request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;
  response->message = msg;

  return;
}

void move_laser_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                         std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values, msg] = move_group_relative("PM_Robot_Laser_TCP",
                                                                       laser_move_group,
                                                                       request->translation,
                                                                       request->rotation,
                                                                       request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;
  response->message = msg;

  return;
}

void move_smarpod_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                           std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values, msg] = move_group_relative("smarpod_endeffector",
                                                                       smarpod_move_group,
                                                                       request->translation,
                                                                       request->rotation,
                                                                       request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;
  response->message = msg;

  return;
}

void move_confocal_head_relative(const std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Request> request,
                                 std::shared_ptr<pm_moveit_interfaces::srv::MoveRelative::Response> response)
{

  auto [success, joint_names, joint_values, msg] = move_group_relative("PM_Robot_Confocal_Head_TCP",
                                                                       confocal_head_move_group,
                                                                       request->translation,
                                                                       request->rotation,
                                                                       request->execute_movement);

  response->success = success;
  response->joint_names = joint_names;
  std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  response->joint_values = joint_values_float;
  response->message = msg;

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

void move_smarpod_to_pose(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Request> request,
                          std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_pose("smarpod_endeffector",
                                                                 smarpod_move_group,
                                                                 request->move_to_pose,
                                                                 request->endeffector_frame_override,
                                                                 request->execute_movement);
  set_initial_state_move_groups();
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

void move_confocal_head_to_pose(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Request> request,
                                std::shared_ptr<pm_moveit_interfaces::srv::MoveToPose::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_pose("PM_Robot_Confocal_Head_TCP",
                                                                 confocal_head_move_group,
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

void move_smarpod_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                           std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_frame("smarpod_endeffector",
                                                                  smarpod_move_group,
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

void move_1k_dispenser_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                                std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_frame("PM_Robot_1K_Dispenser_TCP",
                                                                  dispenser_1k_move_group,
                                                                  "1K_Dispenser_TCP",
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

void move_confocal_head_to_frame(const std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Request> request,
                                 std::shared_ptr<pm_moveit_interfaces::srv::MoveToFrame::Response> response)
{

  auto [success, joint_names, joint_values] = move_group_to_frame("PM_Robot_Confocal_Head_TCP",
                                                                  confocal_head_move_group,
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

void align_gonio_right(const std::shared_ptr<pm_moveit_interfaces::srv::AlignGonio::Request> request,
                       std::shared_ptr<pm_moveit_interfaces::srv::AlignGonio::Response> response)
{

  auto [success, joint_names, joint_values] = align_gonio("PM_Robot_Gonio_Right",
                                                          gonio_right_move_group,
                                                          request->endeffector_frame_override,
                                                          request->target_frame,
                                                          request->rotation_offset_deg,
                                                          request->execute_movement);

  // response->success = success;
  // response->joint_names = joint_names;
  // std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  // response->joint_values = joint_values_float;
  response->success = success;
  return;
}

void align_gonio_left(const std::shared_ptr<pm_moveit_interfaces::srv::AlignGonio::Request> request,
                      std::shared_ptr<pm_moveit_interfaces::srv::AlignGonio::Response> response)
{

  auto [success, joint_names, joint_values] = align_gonio("PM_Robot_Gonio_Left",
                                                          gonio_left_move_group,
                                                          request->endeffector_frame_override,
                                                          request->target_frame,
                                                          request->rotation_offset_deg,
                                                          request->execute_movement);

  // response->success = success;
  // response->joint_names = joint_names;
  // std::vector<float> joint_values_float(joint_values.begin(), joint_values.end());
  // response->joint_values = joint_values_float;
  response->success = success;
  return;
}

void reset_gonio_left(const std::shared_ptr<pm_msgs::srv::EmptyWithSuccess::Request> request,
                      std::shared_ptr<pm_msgs::srv::EmptyWithSuccess::Response> response)
{
  std::vector<double> target_joint_values = {0.0, 0.0};
  auto [move_success, move_msg] = set_move_group(gonio_left_move_group, target_joint_values, true);
  response->success = move_success;
  std::vector<std::string> joint_names = {"Gonio_Left_Stage_1_Joint", "Gonio_Left_Stage_2_Joint"};

  float lateral_tolerance_rougth = 1e-3;
  float angular_tolerance_rougth = 0.1;
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_rougth, angular_tolerance_rougth);

  publish_target_joint_trajectory_gonio_left({0.0, 0.0}, 1);

  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);

  response->success = true;
  return;
}

void reset_gonio_right(const std::shared_ptr<pm_msgs::srv::EmptyWithSuccess::Request> request,
                       std::shared_ptr<pm_msgs::srv::EmptyWithSuccess::Response> response)
{
  std::vector<double> target_joint_values = {0.0, 0.0};
  auto [move_success, move_msg] = set_move_group(gonio_right_move_group, target_joint_values, true);
  std::vector<std::string> joint_names = {"Gonio_Right_Stage_1_Joint", "Gonio_Right_Stage_2_Joint"};

  float lateral_tolerance_rougth = 1e-3;
  float angular_tolerance_rougth = 0.1;
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_rougth, angular_tolerance_rougth);

  publish_target_joint_trajectory_gonio_right({0.0, 0.0}, 1);

  float lateral_tolerance_fine = 1e-6;
  float angular_tolerance_fine = 0.0001;
  wait_for_movement_to_finish(joint_names, target_joint_values, lateral_tolerance_fine, angular_tolerance_fine);

  response->success = true;
  return;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // create a move_to_frame request
  auto move_to_frame_request = std::make_shared<pm_moveit_interfaces::srv::MoveToFrame::Request>();
  default_endeffector_string = move_to_frame_request->endeffector_frame_override;

  pm_moveit_server_node = std::make_shared<rclcpp::Node>(
      "pm_moveit_server",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  use_trajectory_planning = true;
  auto callback_group_re = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions node_options_1;
  node_options_1.callback_group = callback_group_re;

  auto callback_group_me = pm_moveit_server_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions node_options_2;
  node_options_2.callback_group = callback_group_me;

  rclcpp::ExecutorOptions exec_options;
  // exec_options->num_threads = 4;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pm_moveit_server_node);

  auto const logger = rclcpp::get_logger("hello_moveit");

  laser_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Laser_TCP");
  tool_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Tool_TCP");
  Cam1_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Cam1_TCP");
  dispenser_1k_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_1K_Dispenser_TCP");
  gonio_left_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Gonio_Left");
  gonio_right_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Gonio_Right");
  confocal_head_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "PM_Robot_Confocal_Head_TCP");

  get_gonio_right_solution_client = pm_moveit_server_node->create_client<pm_moveit_interfaces::srv::GetGonioSolution>("/gonio_orientation_solver/get_gonio_right_solution");
  get_gonio_left_solution_client = pm_moveit_server_node->create_client<pm_moveit_interfaces::srv::GetGonioSolution>("/gonio_orientation_solver/get_gonio_left_solution");

  plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();

  // auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
  //   pm_moveit_server_node,
  //   "robot_description");

  // // psm->startSceneMonitor("/move_group/monitored_planning_scene");
  // // auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, laser_move_group->getRobotModel());
  // // auto test =  moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node, "world", "moveit_cpp_tutorial", laser_move_group->getRobotModel());
  // psm->requestPlanningSceneState();

  // planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  // const auto& acm = scene->getAllowedCollisionMatrix();

  // scene->getAllowedCollisionMatrixNonConst().setEntry("Calibration_Qube", "PM_Robot_Vacuum_Tool", true);

  // std::vector<std::string> entry_names;
  // acm.getAllEntryNames(entry_names);

  // for (const auto& link1 : entry_names)
  // {
  //   for (const auto& link2 : entry_names)
  //   {
  //     if (link1 >= link2) continue;  // Avoid duplicate pairs and self-pairs

  //     //bool allowed = false;
  //     //acm.getEntry(link1, link2, allowed);

  //     RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Collision allowed between: %s and %s", link1.c_str(), link2.c_str());
  //       //std::cout << "Collision allowed between: " << link1 << " and " << link2 << std::endl;
  //   }
  // }

  laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, laser_move_group->getRobotModel());
  laser_grp_visual_tools->deleteAllMarkers();
  laser_grp_visual_tools->loadRemoteControl();
  // laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(pm_moveit_server_node,"world", rviz_visual_tools::RVIZ_MARKER_TOPIC,laser_move_group->getRobotModel());
  // auto test = moveit_visual_tools::MoveItVisualTools::MoveItVisualTools("/world", rviz_visual_tools::RVIZ_MARKER_TOPIC);
  // auto test = moveit_visual_tools::MoveItVisualTools(pm_moveit_server_node);
  // laser_grp_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(psm);

  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(pm_moveit_server_node, "robot_description");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(pm_moveit_server_node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Dont know why this has to be called at this point, but otherwith the service callbacks get stuck.
  // auto current_state_cam1 = Cam1_move_group->getCurrentState(1.0);
  // auto current_state_tool = tool_move_group->getCurrentState(1.0);
  // auto current_state_laser = laser_move_group->getCurrentState(1.0);

  rclcpp::Service<pm_moveit_interfaces::srv::ExecutePlan>::SharedPtr execute_plan_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::ExecutePlan>("pm_moveit_server/execute_plan", std::bind(&execute_plan, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveCam1TcpTo>::SharedPtr move_cam_one_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveCam1TcpTo>("pm_moveit_server/move_cam1_to_frame", std::bind(&move_cam_one_to_pose,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveToolTcpTo>::SharedPtr move_tool_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToolTcpTo>("pm_moveit_server/move_tool_to_frame", std::bind(&move_group_tool,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);
  // rclcpp::Service<pm_moveit_interfaces::srv::MoveLaserTcpTo>::SharedPtr move_laser_service = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveLaserTcpTo>("pm_moveit_server/move_laser_to_frame", std::bind(&move_group_laser,  std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_cam1_to_frame", std::bind(&move_cam_one_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_tool_to_frame", std::bind(&move_tool_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_laser_to_frame", std::bind(&move_laser_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_1k_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_1k_dispenser_to_frame", std::bind(&move_1k_dispenser_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_confocal_head_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_confocal_head_to_frame", std::bind(&move_confocal_head_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_cam1_relative", std::bind(&move_cam1_relative, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_tool_relative", std::bind(&move_tool_relative, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_laser_relative", std::bind(&move_laser_relative, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_confocal_head_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_confocal_head_relative", std::bind(&move_confocal_head_relative, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);

  auto move_cam_one_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_cam1_to_pose", std::bind(&move_cam_one_to_pose, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_tool_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_tool_to_pose", std::bind(&move_tool_to_pose, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_laser_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_laser_to_pose", std::bind(&move_laser_to_pose, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto move_confocal_head_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_confocal_head_to_pose", std::bind(&move_confocal_head_to_pose, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto reset_gonio_left_srv = pm_moveit_server_node->create_service<pm_msgs::srv::EmptyWithSuccess>("pm_moveit_server/reset_gonio_left", std::bind(&reset_gonio_left, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto reset_gonio_right_srv = pm_moveit_server_node->create_service<pm_msgs::srv::EmptyWithSuccess>("pm_moveit_server/reset_gonio_right", std::bind(&reset_gonio_right, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);

  std::string bringup_package_share_directory = ament_index_cpp::get_package_share_directory("pm_robot_bringup");
  std::string file_path = bringup_package_share_directory + "/config/pm_robot_bringup_config.yaml";

  YAML::Node config = YAML::LoadFile(file_path);
  bool with_smarpod_station = config["pm_smparpod_station"]["with_smarpod_station"].as<bool>();

  // print with_smarpod_station
  // RCLCPP_WARN(rclcpp::get_logger("pm_moveit"), "With Smarpod Station: %s", with_smarpod_station ? "true" : "false");

  rclcpp::Service<pm_moveit_interfaces::srv::MoveToPose>::SharedPtr move_smarpod_to_pose_srv;
  rclcpp::Service<pm_moveit_interfaces::srv::MoveRelative>::SharedPtr move_smarpod_relative_srv;
  rclcpp::Service<pm_moveit_interfaces::srv::MoveToFrame>::SharedPtr move_smarpod_to_frame_srv;

  if (with_smarpod_station)
  {
    smarpod_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(pm_moveit_server_node, "smarpod_endeffector");

    move_smarpod_to_pose_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToPose>("pm_moveit_server/move_smarpod_to_pose", std::bind(&move_smarpod_to_pose, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
    move_smarpod_to_frame_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveToFrame>("pm_moveit_server/move_smarpod_to_frame", std::bind(&move_smarpod_to_frame, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
    move_smarpod_relative_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::MoveRelative>("pm_moveit_server/move_smarpod_relative", std::bind(&move_smarpod_relative, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
    smarpod_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/smaract_hexapod_controller/joint_trajectory", 10);
  }

  auto align_gonio_right_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::AlignGonio>("pm_moveit_server/align_gonio_right", std::bind(&align_gonio_right, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);
  auto align_gonio_left_srv = pm_moveit_server_node->create_service<pm_moveit_interfaces::srv::AlignGonio>("pm_moveit_server/align_gonio_left", std::bind(&align_gonio_left, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_me);

  xyz_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_xyz_axis_controller/joint_trajectory", 10);
  t_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_t_axis_controller/joint_trajectory", 10);
  auto joint_state_subscriber = pm_moveit_server_node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStateCallback, node_options_1);
  gonio_right_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_gonio_right_controller/joint_trajectory", 10);
  gonio_left_trajectory_publisher = pm_moveit_server_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/pm_robot_gonio_left_controller/joint_trajectory", 10);

  RCLCPP_INFO(rclcpp::get_logger("pm_moveit"), "Ready for operation...");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
