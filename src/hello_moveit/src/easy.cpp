/**
 * @file hello_moveit_v2_5dof.cpp
 * @brief MoveIt 2 + ROS 2 code adjusted for a 5-DOF robot arm.
 *
 * The robot only supports yaw and pitch orientation (no roll).
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

  // Define yaw and pitch only (no roll)
  double yaw = 0.0;   // Rotation about Z
  double pitch = 0.0; // Rotation about Y
  double roll = 0.0;  // Fixed for 5-DOF

  // Construct quaternion from limited orientation (5DOF)
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw); // fixed roll
  q.normalize();

  // Set target pose with fixed orientation
  auto const arm_target_pose = [&node, &q]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.214;
    msg.pose.position.y = -0.424;
    msg.pose.position.z = 0.403;
    msg.pose.orientation = tf2::toMsg(q);
    return msg;
  }();

  // Allow orientation tolerance (helps in 5-DOF planning)
  arm_group_interface.setGoalOrientationTolerance(0.1); // ~5.7 degrees

  // Set the target pose
  arm_group_interface.setPoseTarget(arm_target_pose);

  // Plan to target
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute if successful
  if (success) {
    arm_group_interface.execute(plan);
  } else {
    bool ik_success = arm_group_interface.setPoseTarget(arm_target_pose);
    if (!ik_success) {
      RCLCPP_ERROR(logger, "IK failed to set target pose!");
    }
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  RCLCPP_INFO(logger, "Spinning... Node will stay alive. Press Ctrl+C to exit.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

