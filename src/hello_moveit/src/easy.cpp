/**
 * @file hello_moveit_v2_5dof.cpp
 * @brief MoveIt 2 + ROS 2 code adjusted for a 5-DOF robot arm.
 *
 * The robot supports configurable roll, pitch, and yaw orientation.
 * Note: For true 5-DOF robots, one rotational axis is typically constrained.
 */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create MoveGroupInterface for arm
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm_group");
  
  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(10.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());

  // ========================================
  // CONFIGURABLE ORIENTATION VALUES
  // ========================================
  // Change these values as needed (in radians)
  double roll = 0.0;     // Rotation about X-axis
  double pitch = -M_PI/4; // Rotation about Y-axis (example: -45 degrees)
  double yaw = M_PI/6;   // Rotation about Z-axis (example: 30 degrees)
  
  // For convenience, you can also define in degrees and convert:
  // double roll_deg = 0.0;
  // double pitch_deg = -45.0;
  // double yaw_deg = 30.0;
  // double roll = roll_deg * M_PI / 180.0;
  // double pitch = pitch_deg * M_PI / 180.0;
  // double yaw = yaw_deg * M_PI / 180.0;

  RCLCPP_INFO(logger, "Target orientation - Roll: %.3f, Pitch: %.3f, Yaw: %.3f (radians)", 
              roll, pitch, yaw);
  RCLCPP_INFO(logger, "Target orientation - Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°", 
              roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

  // ========================================
  // CONFIGURABLE POSITION VALUES
  // ========================================
  // Change these position values as needed (in meters)
  double target_x = 0.214;
  double target_y = -0.424;
  double target_z = 0.403;

  RCLCPP_INFO(logger, "Target position - X: %.3f, Y: %.3f, Z: %.3f (meters)", 
              target_x, target_y, target_z);

  // Construct quaternion from RPY
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  // Set target pose with configurable position and orientation
  auto const arm_target_pose = [&node, &q, target_x, target_y, target_z]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = target_x;
    msg.pose.position.y = target_y;
    msg.pose.position.z = target_z;
    msg.pose.orientation = tf2::toMsg(q);
    return msg;
  }();

  // Set orientation tolerance (helpful for 5-DOF planning)
  // Increase this value if planning fails due to orientation constraints
  double orientation_tolerance = 0.1; // ~5.7 degrees
  arm_group_interface.setGoalOrientationTolerance(orientation_tolerance);
  
  // Optional: Set position tolerance as well
  double position_tolerance = 0.01; // 1cm
  arm_group_interface.setGoalPositionTolerance(position_tolerance);

  RCLCPP_INFO(logger, "Goal tolerances - Position: %.3fm, Orientation: %.3frad (%.1f°)", 
              position_tolerance, orientation_tolerance, orientation_tolerance * 180.0 / M_PI);

  // Set the target pose
  bool pose_set = arm_group_interface.setPoseTarget(arm_target_pose);
  if (!pose_set) {
    RCLCPP_ERROR(logger, "Failed to set target pose! Check if pose is reachable.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Target pose set successfully. Starting planning...");

  // Plan to target
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute if successful
  if (success) {
    RCLCPP_INFO(logger, "Planning successful! Executing motion...");
    auto execute_result = arm_group_interface.execute(plan);
    
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Motion executed successfully!");
    } else {
      RCLCPP_ERROR(logger, "Motion execution failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed! Possible causes:");
    RCLCPP_ERROR(logger, "  - Target pose is unreachable");
    RCLCPP_ERROR(logger, "  - Orientation constraints too strict for 5-DOF robot");
    RCLCPP_ERROR(logger, "  - Collision detected");
    RCLCPP_ERROR(logger, "Try adjusting target pose or increasing tolerances.");
  }

  RCLCPP_INFO(logger, "Spinning... Node will stay alive. Press Ctrl+C to exit.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
