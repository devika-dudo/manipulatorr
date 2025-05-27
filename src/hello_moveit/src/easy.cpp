/**
 * @file hello_moveit_v2_5dof_topic.cpp
 * @brief MoveIt 2 + ROS 2 code for a 5-DOF robot arm controlled via topics.
 *
 * Subscribes to pose commands and executes them using MoveIt planning.
 * Topics:
 *   - /arm_pose_command (geometry_msgs/PoseStamped): Full pose command
 *   - /arm_pose_rpy_command (geometry_msgs/PoseWithCovarianceStamped): Position + RPY
 */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>

class MoveItTopicController : public rclcpp::Node
{
public:
  MoveItTopicController() : Node("moveit_topic_controller")
  {
    // Declare parameters
    this->declare_parameter("planning_pipeline", "ompl");
    this->declare_parameter("planner_id", "RRTConnectkConfigDefault");
    this->declare_parameter("planning_time", 10.0);
    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("orientation_tolerance", 0.1);
    this->declare_parameter("max_velocity_scaling", 1.0);
    this->declare_parameter("max_acceleration_scaling", 1.0);

    RCLCPP_INFO(this->get_logger(), "MoveIt Topic Controller starting initialization...");
  }

  void initialize()
  {
    // Initialize MoveGroupInterface after construction
    arm_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_group");
    
    // Configure planning parameters
    setupPlanningParameters();

    // Create subscribers for different input methods
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_pose_command", 10,
      std::bind(&MoveItTopicController::poseCallback, this, std::placeholders::_1));

    // Alternative: Subscribe to array of 6 values [x, y, z, roll, pitch, yaw]
    array_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_xyzrpy_command", 10,
      std::bind(&MoveItTopicController::arrayCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveIt Topic Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to topics:");
    RCLCPP_INFO(this->get_logger(), "  - /arm_pose_command (geometry_msgs/PoseStamped)");
    RCLCPP_INFO(this->get_logger(), "  - /arm_xyzrpy_command (std_msgs/Float64MultiArray)");
    RCLCPP_INFO(this->get_logger(), "Planning pipeline: %s", arm_group_interface_->getPlanningPipelineId().c_str());
    RCLCPP_INFO(this->get_logger(), "Planner ID: %s", arm_group_interface_->getPlannerId().c_str());
  }

private:
  void setupPlanningParameters()
  {
    // Get parameters
    std::string planning_pipeline = this->get_parameter("planning_pipeline").as_string();
    std::string planner_id = this->get_parameter("planner_id").as_string();
    double planning_time = this->get_parameter("planning_time").as_double();
    double pos_tol = this->get_parameter("position_tolerance").as_double();
    double orient_tol = this->get_parameter("orientation_tolerance").as_double();
    double max_vel = this->get_parameter("max_velocity_scaling").as_double();
    double max_acc = this->get_parameter("max_acceleration_scaling").as_double();

    // Configure MoveGroupInterface
    arm_group_interface_->setPlanningPipelineId(planning_pipeline);
    arm_group_interface_->setPlannerId(planner_id);
    arm_group_interface_->setPlanningTime(planning_time);
    arm_group_interface_->setGoalPositionTolerance(pos_tol);
    arm_group_interface_->setGoalOrientationTolerance(orient_tol);
    arm_group_interface_->setMaxVelocityScalingFactor(max_vel);
    arm_group_interface_->setMaxAccelerationScalingFactor(max_acc);

    RCLCPP_INFO(this->get_logger(), "Planning configuration:");
    RCLCPP_INFO(this->get_logger(), "  Position tolerance: %.4f m", pos_tol);
    RCLCPP_INFO(this->get_logger(), "  Orientation tolerance: %.4f rad (%.1f°)", 
                orient_tol, orient_tol * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Planning time: %.1f s", planning_time);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    RCLCPP_INFO(this->get_logger(), "Received pose command:");
    RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // Convert quaternion to RPY for logging
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f] rad ([%.1f°, %.1f°, %.1f°])",
                roll, pitch, yaw, 
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    executePoseCommand(*msg);
  }

  void arrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "Expected 6 values [x,y,z,roll,pitch,yaw], got %zu", 
                   msg->data.size());
      return;
    }

    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    double roll = msg->data[3];
    double pitch = msg->data[4];
    double yaw = msg->data[5];

    RCLCPP_INFO(this->get_logger(), "Received XYZ-RPY command:");
    RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", x, y, z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f] rad ([%.1f°, %.1f°, %.1f°])",
                roll, pitch, yaw,
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    // Create pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_link";
    pose_msg.header.stamp = this->now();
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;

    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    pose_msg.pose.orientation = tf2::toMsg(q);

    executePoseCommand(pose_msg);
  }

  void executePoseCommand(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Planning motion to target pose...");

    // Set the target pose
    bool pose_set = arm_group_interface_->setPoseTarget(target_pose);
    if (!pose_set) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set target pose! Check if pose is reachable.");
      return;
    }

    // Plan to target
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group_interface_->plan(plan));

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful! Executing motion...");
      
      auto execute_result = arm_group_interface_->execute(plan);
      
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ Motion executed successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Motion execution failed!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed! Possible causes:");
      RCLCPP_ERROR(this->get_logger(), "  - Target pose is unreachable");
      RCLCPP_ERROR(this->get_logger(), "  - Orientation constraints too strict for 5-DOF robot");
      RCLCPP_ERROR(this->get_logger(), "  - Collision detected");
      RCLCPP_ERROR(this->get_logger(), "  - Joint limits exceeded");
      RCLCPP_ERROR(this->get_logger(), "Try adjusting target pose or increasing tolerances.");
    }
    
    RCLCPP_INFO(this->get_logger(), "Ready for next command...");
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_subscriber_;
  std::mutex planning_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MoveItTopicController>();
  
  // Initialize the node after construction
  node->initialize();
  
  RCLCPP_INFO(node->get_logger(), "🤖 MoveIt Topic Controller ready!");
  RCLCPP_INFO(node->get_logger(), "Send commands to control the 5-DOF robot arm:");
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "Method 1 - Pose message:");
  RCLCPP_INFO(node->get_logger(), "  ros2 topic pub /arm_pose_command geometry_msgs/PoseStamped ...");
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "Method 2 - Array of 6 values [x,y,z,roll,pitch,yaw]:");
  RCLCPP_INFO(node->get_logger(), "  ros2 topic pub /arm_xyzrpy_command std_msgs/Float64MultiArray \"data: [0.3, 0.2, 0.4, 0.0, -0.785, 0.524]\"");
  RCLCPP_INFO(node->get_logger(), " ");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
