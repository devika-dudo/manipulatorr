#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simple_arm_controller");
  
  // Wait for a moment to ensure proper node registration
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(node->get_logger(), "Initializing MoveGroupInterface...");
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm_group");
  
  // Basic configuration - you can adjust these parameters as needed
  move_group->setPlanningTime(10.0);
  move_group->setMaxVelocityScalingFactor(0.3);
  move_group->setMaxAccelerationScalingFactor(0.3);
  move_group->setPlannerId("RRTConnect");
  move_group->setGoalPositionTolerance(0.01);
  move_group->setGoalOrientationTolerance(0.05);
  
  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized");

  // Example 1: Move to a named target position
  RCLCPP_INFO(node->get_logger(), "Moving to home position");
  move_group->setNamedTarget("zero_pose");
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded, executing movement");
    success = (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(node->get_logger(), "Successfully moved to home position");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to execute movement to home position");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan movement to home position");
  }

  // Example 2: Move to a specific pose
  RCLCPP_INFO(node->get_logger(), "Planning movement to target pose");
  
  // Define your target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = -0.509;
  target_pose.position.y = -0.247;
  target_pose.position.z = 0.543;
  target_pose.orientation.x = -0.600;
  target_pose.orientation.y = -0.517;
  target_pose.orientation.z = 0.093;
  target_pose.orientation.w = 0.604;
  
  // Normalize quaternion
  tf2::Quaternion q(
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w
  );
  q.normalize();
  target_pose.orientation = tf2::toMsg(q);
  
  // Set the target pose
  move_group->setPoseTarget(target_pose);
  
  // Plan the motion
  success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node->get_logger(), "Planning succeeded, executing movement to target pose");
    success = (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(node->get_logger(), "Successfully moved to target pose");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to execute movement to target pose");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan movement to target pose");
  }

  // Get current pose
  auto current_pose = move_group->getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "Current pose: position [%f, %f, %f]",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z);
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
