#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <chrono>

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller")
    {
        // Initialize MoveGroupInterface for the single planning group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_group");  // Replace with your actual group name
        
        // Set reference frame
        move_group_->setPoseReferenceFrame("link_6");
        
        // Set planning parameters
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        
        // Initialize height constants
        height_ = 0.18;
        pick_height_ = 0.126;
        carrying_height_ = 0.3;
        init_angle_ = -0.3825;
        
        // Create subscription to target point topic
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_point", 10,
            std::bind(&PandaController::targetPointCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Controller initialized successfully");
    }

private:
    void targetPointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Invalid target point data. Expected at least 3 values [x, y, rotation]");
            return;
        }
        
        double x = msg->data[0];
        double y = msg->data[1];
        double rotation = msg->data[2];
        
        RCLCPP_INFO(this->get_logger(), "Received target point: x=%.3f, y=%.3f, rotation=%.3f", x, y, rotation);
        
        // Execute pick and place sequence
        executePickAndPlace(x, y, rotation);
    }
    
    void executePickAndPlace(double x, double y, double rotation)
    {
        try {
            // 1. Approach: Move to position above the target
            RCLCPP_INFO(this->get_logger(), "Step 1: Moving to approach position");
            moveTo(x, y, height_, 1.0, init_angle_ + rotation, 0.0, 0.0);
            
            // 2. Open gripper
            RCLCPP_INFO(this->get_logger(), "Step 2: Opening gripper");
            gripperAction("open");
            
            // 3. Descend to pick height
            RCLCPP_INFO(this->get_logger(), "Step 3: Descending to pick height");
            moveTo(x, y, pick_height_, 1.0, init_angle_ + rotation, 0.0, 0.0);
            
            // 4. Close gripper to grasp object
            RCLCPP_INFO(this->get_logger(), "Step 4: Closing gripper to grasp object");
            gripperAction("close");
            
            // 5. Lift to carrying height
            RCLCPP_INFO(this->get_logger(), "Step 5: Lifting to carrying height");
            moveTo(x, y, carrying_height_, 1.0, init_angle_ + rotation, 0.0, 0.0);
            
            // 6. Transport to drop-off location
            RCLCPP_INFO(this->get_logger(), "Step 6: Transporting to drop-off location");
            moveTo(0.3, -0.3, carrying_height_, 1.0, init_angle_ + rotation, 0.0, 0.0);
            
            // 7. Open gripper to release object
            RCLCPP_INFO(this->get_logger(), "Step 7: Opening gripper to release object");
            gripperAction("open");
            
            RCLCPP_INFO(this->get_logger(), "Pick and place sequence completed successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during pick and place sequence: %s", e.what());
        }
    }
    
    bool moveTo(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;
        
        move_group_->setPoseTarget(target_pose, "panda_link8");
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing motion");
            move_group_->execute(plan);
            std::this_thread::sleep_for(std::chrono::seconds(3));
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed for position [%.3f, %.3f, %.3f]", x, y, z);
            return false;
        }
    }
    
    bool gripperAction(const std::string& action)
    {
        std::vector<double> joint_group_positions;
        hand_move_group_->getCurrentState()->copyJointGroupPositions(
            hand_move_group_->getCurrentState()->getRobotModel()->getJointModelGroup("hand"),
            joint_group_positions);
        
        if (action == "open") {
            // Open gripper - set both finger joints to 0.04 (max opening)
            joint_group_positions[0] = 10;  // 6th joint
        } else if (action == "close") {
            // Close gripper - set both finger joints to 0.001 (almost closed)
            joint_group_positions[0] = 2;  // 6th joint
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown gripper action: %s", action.c_str());
            return false;
        }
        
        hand_move_group_->setJointValueTarget(joint_group_positions);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (hand_move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Gripper %s planning successful, executing", action.c_str());
            hand_move_group_->execute(plan);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper %s planning failed", action.c_str());
            return false;
        }
    }
    
    // Member variables
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    
    double height_;
    double pick_height_;
    double carrying_height_;
    double init_angle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Create node with MultiThreadedExecutor to handle callbacks
    auto node = std::make_shared<PandaController>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting Panda Controller node...");
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
