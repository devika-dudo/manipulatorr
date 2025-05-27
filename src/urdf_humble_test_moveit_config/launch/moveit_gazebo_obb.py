/**
 * @file pick_and_place.cpp
 * @brief Pick and Place implementation for MoveIt 2 + ROS 2
 * 
 * This node subscribes to object poses from your detection node and performs pick and place operations.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode() : Node("pick_and_place_node")
    {
        // Initialize MoveIt interfaces
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_group");
        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        // Configure MoveIt
        arm_group_->setPlanningPipelineId("ompl");
        arm_group_->setPlannerId("RRTConnectkConfigDefault");
        arm_group_->setPlanningTime(10.0);
        arm_group_->setMaxVelocityScalingFactor(0.5); // Slower for pick/place
        arm_group_->setMaxAccelerationScalingFactor(0.5);
        
        // Set tolerances
        arm_group_->setGoalPositionTolerance(0.01);
        arm_group_->setGoalOrientationTolerance(0.05);
        
        // Subscribe to object poses from your detection node
        object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/cylinder/pose_base", 10,
            std::bind(&PickAndPlaceNode::objectPoseCallback, this, std::placeholders::_1));
        
        // Service or action server for pick and place requests (optional)
        pick_place_timer_ = this->create_timer(
            std::chrono::seconds(1),
            std::bind(&PickAndPlaceNode::checkForPickAndPlace, this));
        
        RCLCPP_INFO(this->get_logger(), "Pick and Place Node initialized");
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", 
                   arm_group_->getEndEffectorLink().c_str());
        
        // Define some common poses
        setupCommonPoses();
    }

private:
    // MoveIt interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_pose_sub_;
    rclcpp::TimerBase::SharedPtr pick_place_timer_;
    
    // State
    geometry_msgs::msg::PointStamped latest_object_pose_;
    bool object_detected_ = false;
    bool pick_place_in_progress_ = false;
    
    // Common poses
    geometry_msgs::msg::Pose home_pose_;
    geometry_msgs::msg::Pose drop_pose_;

    void setupCommonPoses()
    {
        // Home position (safe position)
        home_pose_.position.x = 0.3;
        home_pose_.position.y = 0.0;
        home_pose_.position.z = 0.5;
        
        tf2::Quaternion q_home;
        q_home.setRPY(0, -M_PI/2, 0); // Looking down
        home_pose_.orientation = tf2::toMsg(q_home);
        
        // Drop position
        drop_pose_.position.x = -0.3;
        drop_pose_.position.y = 0.3;
        drop_pose_.position.z = 0.3;
        drop_pose_.orientation = home_pose_.orientation;
        
        RCLCPP_INFO(this->get_logger(), "Common poses configured");
    }

    

    void checkForPickAndPlace()
    {
        if (object_detected_ && !pick_place_in_progress_) {
            RCLCPP_INFO(this->get_logger(), "Starting pick and place operation");
            pick_place_in_progress_ = true;
            
            if (executePickAndPlace()) {
                RCLCPP_INFO(this->get_logger(), "Pick and place completed successfully!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pick and place failed!");
            }
            
            pick_place_in_progress_ = false;
            object_detected_ = false; // Reset for next object
        }
    }

    bool executePickAndPlace()
    {
        // Step 1: Go to home position
        if (!moveToTarget(home_pose_, "home position")) {
            return false;
        }
        
        // Step 2: Open gripper
        if (!openGripper()) {
            return false;
        }
        
        // Step 3: Move to pre-pick position (above object)
        geometry_msgs::msg::Pose pre_pick_pose = latest_object_pose_.pose;
        pre_pick_pose.position.z += 0.15; // 15cm above object
        
        if (!moveToTarget(pre_pick_pose, "pre-pick position")) {
            return false;
        }
        
        // Step 4: Move down to pick position
        geometry_msgs::msg::Pose pick_pose = latest_object_pose_.pose;
        pick_pose.position.z += 0.02; // 2cm above object (adjust based on gripper)
        
        // Ensure proper orientation for picking
        tf2::Quaternion q_pick;
        q_pick.setRPY(0, -M_PI/2, 0); // Looking down
        pick_pose.orientation = tf2::toMsg(q_pick);
        
        if (!moveToTarget(pick_pose, "pick position")) {
            return false;
        }
        
        // Step 5: Close gripper
        if (!closeGripper()) {
            return false;
        }
        
        // Step 6: Lift object
        geometry_msgs::msg::Pose lift_pose = pick_pose;
        lift_pose.position.z += 0.1; // Lift 10cm
        
        if (!moveToTarget(lift_pose, "lift position")) {
            return false;
        }
        
        // Step 7: Move to drop position (above)
        geometry_msgs::msg::Pose pre_drop_pose = drop_pose_;
        pre_drop_pose.position.z += 0.1; // 10cm above drop
        
        if (!moveToTarget(pre_drop_pose, "pre-drop position")) {
            return false;
        }
        
        // Step 8: Move down to drop position
        if (!moveToTarget(drop_pose_, "drop position")) {
            return false;
        }
        
        // Step 9: Open gripper to release object
        if (!openGripper()) {
            return false;
        }
        
        // Step 10: Move up from drop position
        geometry_msgs::msg::Pose post_drop_pose = drop_pose_;
        post_drop_pose.position.z += 0.1;
        
        if (!moveToTarget(post_drop_pose, "post-drop position")) {
            return false;
        }
        
        // Step 11: Return to home
        return moveToTarget(home_pose_, "home position");
    }
    void PickAndPlaceNode::objectPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    geometry_msgs::msg::Pose target_pose;

    // Use position from the detected point
    target_pose.position.x = msg->point.x;
    target_pose.position.y = msg->point.y;
    target_pose.position.z = msg->point.z + 0.05;  // Optional offset for grasping

    // Set a fixed orientation for picking (e.g., top-down)
    tf2::Quaternion orientation;
    orientation.setRPY(0, M_PI, 0);  // Flip gripper if needed
    target_pose.orientation = tf2::toMsg(orientation);

    // Save this pose or send it directly to move_group
    latest_pick_pose_ = target_pose;
    has_new_pose_ = true;

    RCLCPP_INFO(this->get_logger(), "Received object position: (%.3f, %.3f, %.3f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
}

    bool moveToTarget(const geometry_msgs::msg::Pose& target_pose, const std::string& description)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to %s", description.c_str());
        
        arm_group_->setPoseTarget(target_pose);
        
        auto const [success, plan] = [&] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(arm_group_->plan(msg));
            return std::make_pair(ok, msg);
        }();
        
        if (success) {
            auto execute_result = arm_group_->execute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Successfully moved to %s", description.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute motion to %s", description.c_str());
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan motion to %s", description.c_str());
            return false;
        }
    }

    bool openGripper()
{
    RCLCPP_INFO(this->get_logger(), "Opening gripper");
    
    // Set joint_6 to maximum value (10) to open gripper
    return setGripperJoint(10.0);
}

bool closeGripper()
{
    RCLCPP_INFO(this->get_logger(), "Closing gripper");
    
    // Set joint_6 to minimum value (0) to close gripper
    return setGripperJoint(0.0);
}

bool setGripperJoint(double joint_6_value)
{
    if (!arm_group_) {
        RCLCPP_WARN(this->get_logger(), "No arm group available");
        return false;
    }

    // Clamp value between 0 and 10
    joint_6_value = std::max(0.0, std::min(10.0, joint_6_value));

    std::vector<double> joint_values = arm_group_->getCurrentJointValues();

    if (joint_values.size() < 6) {
        RCLCPP_ERROR(this->get_logger(), "Expected at least 6 joints, but got %zu", joint_values.size());
        return false;
    }

    joint_values[5] = joint_6_value;  // Assuming joint_6 is index 5

    arm_group_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group_->plan(plan));
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan gripper motion");
        return false;
    }

    auto result = arm_group_->execute(plan);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
}


// Optional: Function to set gripper to a specific position between 0-10
bool setGripperPosition(double position)
{
    RCLCPP_INFO(this->get_logger(), "Setting gripper to position: %.2f", position);
    return setGripperJoint(position);
}
    void addCollisionObject(const std::string& object_id, 
                           const geometry_msgs::msg::Pose& pose,
                           const std::vector<double>& dimensions)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = arm_group_->getPlanningFrame();
        collision_object.id = object_id;
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = dimensions[0]; // x
        primitive.dimensions[1] = dimensions[1]; // y
        primitive.dimensions[2] = dimensions[2]; // z
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_->addCollisionObjects(collision_objects);
        
        RCLCPP_INFO(this->get_logger(), "Added collision object: %s", object_id.c_str());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();
    
    RCLCPP_INFO(node->get_logger(), "Pick and Place node spinning...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
