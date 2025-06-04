/**
 * @file hello_moveit_v2_5dof_constrained.cpp
 * @brief MoveIt 2 + ROS 2 code for a 5-DOF robot arm with fake joint filtering.
 *
 * This version filters out the fake joint from 6-DOF planning before 
 * sending 5-DOF commands to the actual robot controller.
 */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>

class MoveIt5DOFController : public rclcpp::Node
{
public:
  MoveIt5DOFController() : Node("moveit_5dof_controller")
  {
    // Declare parameters
    this->declare_parameter("planning_pipeline", "ompl");
    this->declare_parameter("planner_id", "RRTConnectkConfigDefault");
    this->declare_parameter("planning_time", 10.0);
    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("orientation_tolerance", 0.2);
    this->declare_parameter("max_velocity_scaling", 0.5);
    this->declare_parameter("max_acceleration_scaling", 0.5);
    this->declare_parameter("use_constraints", true);
    this->declare_parameter("constrain_orientation", true);
    
    // IMPORTANT: Name of the fake joint to filter out
    this->declare_parameter("fake_joint_name", "fake_joint");  // Change this to your fake joint name

    RCLCPP_INFO(this->get_logger(), "MoveIt 5DOF Controller starting initialization...");
  }

  void initialize()
  {
    // Initialize MoveGroupInterface after construction
    arm_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_group");
    
    // Get joint names and identify fake joint index
    setupJointFiltering();
    
    // Configure planning parameters
    setupPlanningParameters();

    // Create publisher for filtered 5-DOF joint commands
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_group_controller/joint_trajectory", 10);

    // Create subscribers for different input methods
    position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/arm_position_command", 10,
      std::bind(&MoveIt5DOFController::positionCallback, this, std::placeholders::_1));

    xyz_rp_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_xyz_rpy_command", 10,  // Changed topic name to reflect 6DOF
      std::bind(&MoveIt5DOFController::xyzRpCallback, this, std::placeholders::_1));

    pose_constrained_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_pose_constrained_command", 10,
      std::bind(&MoveIt5DOFController::poseConstrainedCallback, this, std::placeholders::_1));

    joint_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_joint_command", 10,
      std::bind(&MoveIt5DOFController::jointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveIt 5DOF Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Fake joint '%s' will be filtered out (index: %d)", 
                fake_joint_name_.c_str(), fake_joint_index_);
  }

private:
  void setupJointFiltering()
  {
    // Get all joint names from MoveGroup
    std::vector<std::string> joint_names = arm_group_interface_->getJointNames();
    
    RCLCPP_INFO(this->get_logger(), "MoveGroup joint names (%zu joints):", joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  [%zu]: %s", i, joint_names[i].c_str());
    }

    // Get fake joint name parameter
    fake_joint_name_ = this->get_parameter("fake_joint_name").as_string();
    
    // Find fake joint index
    fake_joint_index_ = -1;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] == fake_joint_name_) {
        fake_joint_index_ = static_cast<int>(i);
        break;
      }
    }

    if (fake_joint_index_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "Fake joint '%s' not found in joint names!", fake_joint_name_.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available joints:");
      for (const auto& name : joint_names) {
        RCLCPP_ERROR(this->get_logger(), "  - %s", name.c_str());
      }
      return;
    }

    // Create filtered joint names (without fake joint)
    real_joint_names_.clear();
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (static_cast<int>(i) != fake_joint_index_) {
        real_joint_names_.push_back(joint_names[i]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Real joint names (%zu joints):", real_joint_names_.size());
    for (size_t i = 0; i < real_joint_names_.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  [%zu]: %s", i, real_joint_names_[i].c_str());
    }
  }

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

    // Clear any existing constraints
    arm_group_interface_->clearPathConstraints();
  }

  // Filter out fake joint from joint trajectory
  trajectory_msgs::msg::JointTrajectory filterTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    trajectory_msgs::msg::JointTrajectory filtered_traj;
    
    // Copy header
    filtered_traj.header = plan.trajectory_.joint_trajectory.header;
    
    // Set filtered joint names
    filtered_traj.joint_names = real_joint_names_;
    
    // Filter trajectory points
    for (const auto& point : plan.trajectory_.joint_trajectory.points) {
      trajectory_msgs::msg::JointTrajectoryPoint filtered_point;
      
      // Filter positions
      for (size_t i = 0; i < point.positions.size(); ++i) {
        if (static_cast<int>(i) != fake_joint_index_) {
          filtered_point.positions.push_back(point.positions[i]);
        }
      }
      
      // Filter velocities
      if (!point.velocities.empty()) {
        for (size_t i = 0; i < point.velocities.size(); ++i) {
          if (static_cast<int>(i) != fake_joint_index_) {
            filtered_point.velocities.push_back(point.velocities[i]);
          }
        }
      }
      
      // Filter accelerations
      if (!point.accelerations.empty()) {
        for (size_t i = 0; i < point.accelerations.size(); ++i) {
          if (static_cast<int>(i) != fake_joint_index_) {
            filtered_point.accelerations.push_back(point.accelerations[i]);
          }
        }
      }
      
      // Copy time
      filtered_point.time_from_start = point.time_from_start;
      
      filtered_traj.points.push_back(filtered_point);
    }
    
    return filtered_traj;
  }

  // Method 1: Position-only control
  void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    RCLCPP_INFO(this->get_logger(), "Received position-only command:");
    RCLCPP_INFO(this->get_logger(), "  Target position: [%.3f, %.3f, %.3f]", 
                msg->point.x, msg->point.y, msg->point.z);

    arm_group_interface_->clearPathConstraints();
    arm_group_interface_->clearPoseTargets();
    
    bool success = arm_group_interface_->setPositionTarget(
      msg->point.x, msg->point.y, msg->point.z);
    
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position target!");
      return;
    }

    planAndExecuteFiltered("position target");
  }

  // Method 2: Position + Roll/Pitch/Yaw (now accepts 6 values: x,y,z,roll,pitch,yaw)
  void xyzRpCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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

    RCLCPP_INFO(this->get_logger(), "Received XYZ + RPY command:");
    RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", x, y, z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw);

    if (this->get_parameter("use_constraints").as_bool()) {
      executeWithOrientationConstraints(x, y, z, roll, pitch);
    } else {
      executeWithApproximatePose(x, y, z, roll, pitch, yaw);
    }
  }

  // Method 3: Full pose with constraints
  void poseConstrainedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    RCLCPP_INFO(this->get_logger(), "Received constrained pose command");
    
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    executeWithOrientationConstraints(x, y, z, roll, pitch);
  }

  // Method 4: Direct joint control (accepts both 5 and 6 joints)
  void jointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    std::vector<double> joint_target;
    
    if (msg->data.size() == 5) {
      // 5-DOF input: insert fake joint value
      RCLCPP_INFO(this->get_logger(), "Received 5-DOF joint command:");
      RCLCPP_INFO(this->get_logger(), "  Joints: [%.3f, %.3f, %.3f, %.3f, %.3f]", 
                  msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
      
      for (size_t i = 0; i < msg->data.size() + 1; ++i) {
        if (static_cast<int>(i) == fake_joint_index_) {
          joint_target.push_back(0.0); // Fake joint value
        } else {
          size_t real_index = (i < fake_joint_index_) ? i : i - 1;
          joint_target.push_back(msg->data[real_index]);
        }
      }
    } 
    else if (msg->data.size() == 6) {
      // 6-DOF input: use directly
      RCLCPP_INFO(this->get_logger(), "Received 6-DOF joint command:");
      RCLCPP_INFO(this->get_logger(), "  Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                  msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
      joint_target = msg->data;
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Expected 5 or 6 joint values, got %zu", msg->data.size());
      return;
    }

    arm_group_interface_->clearPathConstraints();
    arm_group_interface_->setJointValueTarget(joint_target);
    
    planAndExecuteFiltered("joint target");
  }

  void executeWithOrientationConstraints(double x, double y, double z, double roll, double pitch)
  {
    arm_group_interface_->clearPathConstraints();
    arm_group_interface_->clearPoseTargets();
    arm_group_interface_->setPositionTarget(x, y, z);

    // Create orientation constraints for roll and pitch only
    moveit_msgs::msg::Constraints constraints;
    
    // Roll constraint
    moveit_msgs::msg::OrientationConstraint roll_constraint;
    roll_constraint.link_name = arm_group_interface_->getEndEffectorLink();
    roll_constraint.header.frame_id = arm_group_interface_->getPlanningFrame();
    
    tf2::Quaternion roll_q;
    roll_q.setRPY(roll, 0, 0);
    roll_constraint.orientation = tf2::toMsg(roll_q);
    roll_constraint.absolute_x_axis_tolerance = 0.1;
    roll_constraint.absolute_y_axis_tolerance = 3.14;
    roll_constraint.absolute_z_axis_tolerance = 3.14;
    roll_constraint.weight = 1.0;
    
    // Pitch constraint
    moveit_msgs::msg::OrientationConstraint pitch_constraint;
    pitch_constraint.link_name = arm_group_interface_->getEndEffectorLink();
    pitch_constraint.header.frame_id = arm_group_interface_->getPlanningFrame();
    
    tf2::Quaternion pitch_q;
    pitch_q.setRPY(0, pitch, 0);
    pitch_constraint.orientation = tf2::toMsg(pitch_q);
    pitch_constraint.absolute_x_axis_tolerance = 3.14;
    pitch_constraint.absolute_y_axis_tolerance = 0.1;
    pitch_constraint.absolute_z_axis_tolerance = 3.14;
    pitch_constraint.weight = 1.0;

    constraints.orientation_constraints.push_back(roll_constraint);
    constraints.orientation_constraints.push_back(pitch_constraint);
    
    arm_group_interface_->setPathConstraints(constraints);
    
    planAndExecuteFiltered("position + orientation constraints");
  }

  void executeWithApproximatePose(double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = arm_group_interface_->getPlanningFrame();
    target_pose.header.stamp = this->now();
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    target_pose.pose.orientation = tf2::toMsg(q);

    arm_group_interface_->clearPathConstraints();
    
    double original_orient_tol = this->get_parameter("orientation_tolerance").as_double();
    arm_group_interface_->setGoalOrientationTolerance(0.5);
    
    bool success = arm_group_interface_->setPoseTarget(target_pose);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set pose target!");
      return;
    }

    planAndExecuteFiltered("approximate pose");
    
    arm_group_interface_->setGoalOrientationTolerance(original_orient_tol);
  }

  void planAndExecuteFiltered(const std::string& method_name)
  {
    RCLCPP_INFO(this->get_logger(), "Planning motion using %s...", method_name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group_interface_->plan(plan));

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful! Filtering and executing...");
      
      // Filter out fake joint from trajectory
      trajectory_msgs::msg::JointTrajectory filtered_traj = filterTrajectory(plan);
      
      RCLCPP_INFO(this->get_logger(), "Original trajectory: %zu joints, %zu points", 
                  plan.trajectory_.joint_trajectory.joint_names.size(),
                  plan.trajectory_.joint_trajectory.points.size());
      RCLCPP_INFO(this->get_logger(), "Filtered trajectory: %zu joints, %zu points", 
                  filtered_traj.joint_names.size(),
                  filtered_traj.points.size());
      
      // Publish filtered trajectory to robot controller
      joint_trajectory_pub_->publish(filtered_traj);
      
      RCLCPP_INFO(this->get_logger(), "✅ Filtered trajectory sent to controller!");
      
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed with %s", method_name.c_str());
      RCLCPP_ERROR(this->get_logger(), "Try different target or check workspace limits");
    }
    
    RCLCPP_INFO(this->get_logger(), "Ready for next command...");
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr xyz_rp_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_constrained_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_subscriber_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
  
  std::string fake_joint_name_;
  int fake_joint_index_;
  std::vector<std::string> real_joint_names_;
  std::mutex planning_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MoveIt5DOFController>();
  node->initialize();
  
  RCLCPP_INFO(node->get_logger(), "🤖 MoveIt 5DOF Controller with Fake Joint Filter ready!");
  RCLCPP_INFO(node->get_logger(), "");
  RCLCPP_INFO(node->get_logger(), "📡 Publishing filtered 5-DOF trajectories to:");
  RCLCPP_INFO(node->get_logger(), "  /arm_controller/joint_trajectory");
  RCLCPP_INFO(node->get_logger(), "");
  RCLCPP_INFO(node->get_logger(), "🎯 Control methods:");
  RCLCPP_INFO(node->get_logger(), "  1. Position: /arm_position_command");
  RCLCPP_INFO(node->get_logger(), "  2. XYZ+RP: /arm_xyz_rp_command");
  RCLCPP_INFO(node->get_logger(), "  3. Joints: /arm_joint_command (5 joints)");
  
  rclcpp::spin(node);
  rclcpp::shutdown();  
  return 0;
}
