// arm_control_from_UI.cpp
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

class ArmControlFromUI : public rclcpp::Node
{
public:
  ArmControlFromUI(const rclcpp::NodeOptions& options)
  : Node("arm_control_from_ui", options)
  {
    // Create subscription to receive coordinates from UI
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/target_point", 10, std::bind(&ArmControlFromUI::targetPointCallback, this, _1));

    // Allow MoveIt to fully initialize
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces...");
    
    // Create executor for the MoveIt node
    moveit_node_ = std::make_shared<rclcpp::Node>("moveit_node", options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(moveit_node_);
    
    // Start the executor thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    
    // Initialize MoveGroupInterface with the dedicated node
    try {
      move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, "arm_group");
      
      // Configure the planner
      move_group_ptr_->setPlanningTime(10.0);
      move_group_ptr_->setMaxVelocityScalingFactor(0.5);
      move_group_ptr_->setMaxAccelerationScalingFactor(0.5);
      move_group_ptr_->setGoalPositionTolerance(0.01);
      move_group_ptr_->setGoalOrientationTolerance(0.01);
      move_group_ptr_->setNumPlanningAttempts(5);
      move_group_ptr_->allowReplanning(true);
      
      // Try to initialize hand group if available
      try {
        hand_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          moveit_node_, "hand");
        has_hand_group_ = true;
        RCLCPP_INFO(this->get_logger(), "Hand group initialized successfully");
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Hand group not available: %s", e.what());
        has_hand_group_ = false;
      }
      
      RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interfaces: %s", e.what());
      throw;
    }
  }

  ~ArmControlFromUI()
  {
    // Stop the executor and join the thread
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

private:
  void targetPointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Received target point with insufficient data");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received target point: [%f, %f, %f]", 
                msg->data[0], msg->data[1], msg->data[2]);
    
    // Extract target coordinates
    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    
    // Get orientation from additional parameters if available
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    
    if (msg->data.size() >= 7) {
      qx = msg->data[3];
      qy = msg->data[4];
      qz = msg->data[5];
      qw = msg->data[6];
    }
    
    // Move to the target position
    moveToTarget(x, y, z, qx, qy, qz, qw);
  }

  bool moveToTarget(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    // Reset any previous targets
    move_group_ptr_->clearPoseTargets();
    
    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;
    
    // Set the target pose
    RCLCPP_INFO(this->get_logger(), "Setting target pose: [%f, %f, %f]", x, y, z);
    move_group_ptr_->setPoseTarget(target_pose);
    
    // Create a plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = false;
    int max_attempts = 3;
    
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
      RCLCPP_INFO(this->get_logger(), "Planning attempt %d/%d...", attempt, max_attempts);
      
      try {
        auto plan_result = move_group_ptr_->plan(plan);
        success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
          RCLCPP_INFO(this->get_logger(), "Planning successful!");
          break;
        } else {
          RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed", attempt);
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning attempt %d: %s", 
                    attempt, e.what());
      }
      
      if (attempt < max_attempts) {
        RCLCPP_INFO(this->get_logger(), "Waiting before retry...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing plan...");
      
      try {
        moveit::core::MoveItErrorCode execute_result = move_group_ptr_->execute(plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Execution complete!");
          return true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Execution failed");
          return false;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during execution: %s", e.what());
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed after %d attempts!", max_attempts);
      return false;
    }
  }
  
  // Method to control gripper (if available)
  bool controlGripper(bool open)
  {
    if (!has_hand_group_) {
      RCLCPP_WARN(this->get_logger(), "Hand group not available for gripper control");
      return false;
    }
    
    // Set appropriate joint value for open/close
    double joint_value = open ? 5.3 : 1.0;  // Example values, adjust based on your gripper
    RCLCPP_INFO(this->get_logger(), "Setting gripper to %s (value: %f)", 
               open ? "open" : "close", joint_value);
    
    // Get joint names and set the target
    const auto& joint_names = hand_group_ptr_->getJointNames();
    if (joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No joints found for hand group");
      return false;
    }
    
    hand_group_ptr_->setJointValueTarget(joint_names[0], joint_value);
    
    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (hand_group_ptr_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing gripper movement");
      auto execute_result = hand_group_ptr_->execute(plan);
      return (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Gripper planning failed");
      return false;
    }
  }

  // Node for MoveGroupInterface
  rclcpp::Node::SharedPtr moveit_node_;
  
  // Executor for the MoveIt node
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  
  // Thread for the executor
  std::thread executor_thread_;
  
  // MoveGroup interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_ptr_;
  bool has_hand_group_ = false;
  
  // Subscription for target points from UI
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  try {
    auto arm_control_node = std::make_shared<ArmControlFromUI>(node_options);
    rclcpp::spin(arm_control_node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_control_from_ui"), "Error: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
