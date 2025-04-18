#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <cmath>
#include <iomanip>

// Define a class for the MoveIt2 Interface
class MoveitInterface : public rclcpp::Node
{
public:
    MoveitInterface(const rclcpp::NodeOptions &options);
    ~MoveitInterface();
    
    // Method to plan and execute movement
    bool planAndExecuteMovement(const geometry_msgs::msg::Pose& target_pose);
    
    // Method to get current pose of hand
    geometry_msgs::msg::PoseStamped getCurrentPose();
    
    // Method to move to a named state
    bool moveToNamedState(const std::string& state_name);
    
    // Debugging methods
    void displayPlanningSceneBoundingBox();
    void displayAvailableNamedStates();
    void displayJointLimits();
    
private:
    // Node for MoveGroupInterface
    rclcpp::Node::SharedPtr moveit_node_;
    
    // Executor for the MoveIt node
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // Thread for the executor
    std::thread executor_thread_;
    
    // MoveGroup interface
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
    
    // End effector name
    const std::string end_effector_link_{"link_5"};
};

// Implement constructor
MoveitInterface::MoveitInterface(const rclcpp::NodeOptions &options)
    : Node("hello_moveit", options),
      moveit_node_(std::make_shared<rclcpp::Node>("moveit_node")),
      executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
    // Add node to executor and start spinning in a separate thread
    executor_->add_node(moveit_node_);
    
    // Initialize MoveGroupInterface with the dedicated node
    move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, 
        "arm_group"
    );
    
    // Set the end effector link
    move_group_ptr_->setEndEffectorLink(end_effector_link_);
    
    // Configure the planner
    move_group_ptr_->setPlanningPipelineId("ompl");
    move_group_ptr_->setPlanningTime(20.0);
    move_group_ptr_->setMaxVelocityScalingFactor(0.3);
    move_group_ptr_->setMaxAccelerationScalingFactor(0.3);
    move_group_ptr_->setGoalPositionTolerance(0.02);
    move_group_ptr_->setGoalOrientationTolerance(0.02);
    move_group_ptr_->startStateMonitor();
    
    // Start spinning in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    
    RCLCPP_INFO(this->get_logger(), "MoveitInterface initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using '%s' as end effector link", end_effector_link_.c_str());
}

// Implement destructor
MoveitInterface::~MoveitInterface()
{
    // Stop the executor and join the thread
    executor_->cancel();
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
}

// Implement method to plan and execute movement
bool MoveitInterface::planAndExecuteMovement(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_ptr_->setGoalTolerance(0.01); // Set the tolerance to 1 cm
    move_group_ptr_->setApproximateJointValueTarget(target_pose, end_effector_link_);

    // Create a plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_ptr_->plan(plan));
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful! Executing plan...");
        move_group_ptr_->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Execution complete!");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        return false;
    }
}

// Implement method to get current pose
geometry_msgs::msg::PoseStamped MoveitInterface::getCurrentPose()
{
    return move_group_ptr_->getCurrentPose(end_effector_link_);
}

// Implement method to move to a named state
bool MoveitInterface::moveToNamedState(const std::string& state_name)
{
    move_group_ptr_->setNamedTarget(state_name);
    
    // Create a plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_ptr_->plan(plan));
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning to %s successful! Executing plan...", state_name.c_str());
        move_group_ptr_->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Execution complete!");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning to %s failed!", state_name.c_str());
        return false;
    }
}

// Debugging method to display planning scene bounding box
void MoveitInterface::displayPlanningSceneBoundingBox()
{
    try {
        auto current_state = move_group_ptr_->getCurrentState();
        if (current_state) {
            std::vector<double> aabb(6); // Create a vector to hold the AABB values
            current_state->computeAABB(aabb); // Call with the vector
            
            // Use the values from the aabb vector directly
            double minX = aabb[0], minY = aabb[1], minZ = aabb[2];
            double maxX = aabb[3], maxY = aabb[4], maxZ = aabb[5];
            
            RCLCPP_INFO(this->get_logger(), "Planning scene bounds:");
            RCLCPP_INFO(this->get_logger(), "  Min: [%f, %f, %f]", minX, minY, minZ);
            RCLCPP_INFO(this->get_logger(), "  Max: [%f, %f, %f]", maxX, maxY, maxZ);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute planning scene bounds: %s", e.what());
    }
}

// Debugging method to display available named states
void MoveitInterface::displayAvailableNamedStates()
{
    auto available_states = move_group_ptr_->getNamedTargets();
    RCLCPP_INFO(this->get_logger(), "Available named states:");
    if (available_states.empty()) {
        RCLCPP_INFO(this->get_logger(), "  No named states available");
    } else {
        for (const auto& state : available_states) {
            RCLCPP_INFO(this->get_logger(), "  - %s", state.c_str());
        }
    }
}

// Debugging method to display joint limits
void MoveitInterface::displayJointLimits()
{
    auto joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup("arm_group");
    if (joint_model_group) {
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        RCLCPP_INFO(this->get_logger(), "Joint limits for group %s:", "arm_group");
        
        for (const auto& name : joint_names) {
            const auto* joint_model = move_group_ptr_->getCurrentState()->getJointModel(name);
            if (joint_model && joint_model->getType() != moveit::core::JointModel::UNKNOWN && 
                joint_model->getType() != moveit::core::JointModel::FIXED) {
                auto bounds = joint_model->getVariableBounds(joint_model->getName()); // Get the VariableBounds object
                if (bounds.position_bounded_) { // Check if position is bounded
                    RCLCPP_INFO(this->get_logger(), "  %s: [%f, %f]", 
                                name.c_str(), bounds.min_position_, bounds.max_position_);
                }
            }
        }
    }
}

// Helper function for rounding
double round_to_decimal(double value, int decimal_places) {
    double factor = std::pow(10, decimal_places);
    return std::round(value * factor) / factor;
}

// Global flag for logging
std::atomic<bool> logging_active(true);

// Function to log the hand position
void log_hand_position(MoveitInterface& moveit_interface) {
    std::ofstream csv_file("hand_position_log.csv");
    csv_file << "Time;X;Y;Z\n";
    csv_file << std::fixed << std::setprecision(4);
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (logging_active) {
        try {
            auto current_pose = moveit_interface.getCurrentPose();
            auto now = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(now - start_time).count();
            
            csv_file << elapsed_seconds << ";"
                     << round_to_decimal(current_pose.pose.position.x * 1000, 4) << ";"
                     << round_to_decimal(current_pose.pose.position.y * 1000, 4) << ";"
                     << round_to_decimal(current_pose.pose.position.z * 1000, 4) << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Could not get current pose: " << e.what() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Log every 100 ms
    }
    csv_file.close();
}

int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create node options
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // Create the MoveitInterface instance
    auto moveit_interface = std::make_shared<MoveitInterface>(node_options);
    
    // Give the system time to establish connections
    RCLCPP_INFO(moveit_interface->get_logger(), "Waiting for connections to establish...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Display debugging information
    moveit_interface->displayAvailableNamedStates();
    moveit_interface->displayPlanningSceneBoundingBox();
    moveit_interface->displayJointLimits();
    
    // Try to print the current pose to verify access to robot state
    try {
        auto current_pose = moveit_interface->getCurrentPose();
        RCLCPP_INFO(moveit_interface->get_logger(), 
                   "Current position: x=%f, y=%f, z=%f", 
                   current_pose.pose.position.x,
                   current_pose.pose.position.y,
                   current_pose.pose.position.z);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(moveit_interface->get_logger(), "Exception while getting robot state: %s", e.what());
        return 1;
    }
    
    // Define the target pose directly
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = -0.509;  // X position
    target_pose.position.y = -0.247;  // Y position
    target_pose.position.z = 0.543;    // Z position
    
    // Set the orientation using the provided quaternion
    target_pose.orientation.x = -0.600;
    target_pose.orientation.y = -0.517;
    target_pose.orientation.z = 0.093;
    target_pose.orientation.w = 0.604;

    // Log the target pose
    RCLCPP_INFO(moveit_interface->get_logger(), 
               "Planning trajectory to target: x=%f, y=%f, z=%f", 
               target_pose.position.x,
               target_pose.position.y,
               target_pose.position.z);
    
    // Plan and execute movement to the target pose
    bool success = moveit_interface->planAndExecuteMovement(target_pose);
    
    if (!success) {
        RCLCPP_ERROR(moveit_interface->get_logger(), "Failed to move to the target pose.");
    } else {
        RCLCPP_INFO(moveit_interface->get_logger(), "Successfully moved to the target pose.");
    }
    rclcpp::sleep_for(std::chrono::seconds(5)); // Keep the node alive for 5 seconds
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
