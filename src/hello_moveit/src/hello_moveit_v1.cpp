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
    geometry_msgs::msg::PoseStamped getCurrentPose(int max_retries = 5);
    
    // Method to move to a named state
    bool moveToNamedState(const std::string& state_name);
    
    // Debugging methods
    void displayPlanningSceneBoundingBox();
    void displayAvailableNamedStates();
    void displayJointLimits();
    void waitForRobotState(double timeout_seconds = 10.0);
    
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
    
    // Joint state subscriber to check if we're receiving data
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::atomic<bool> received_joint_state_{false};
    
    // Callback for joint states
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

// Implement constructor
MoveitInterface::MoveitInterface(const rclcpp::NodeOptions &options)
    : Node("hello_moveit", options),
      moveit_node_(std::make_shared<rclcpp::Node>("moveit_node", options)),
      executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
    // Make sure sim time parameter is consistent
    bool use_sim_time = true;  // Default to true for simulation
    
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", use_sim_time);
    } else {
        this->get_parameter("use_sim_time", use_sim_time);
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting use_sim_time = %s for both nodes", 
               use_sim_time ? "true" : "false");
               
    // Set the parameter for the MoveIt node too
    if (!moveit_node_->has_parameter("use_sim_time")) {
        moveit_node_->declare_parameter("use_sim_time", use_sim_time);
    } else {
        moveit_node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    }
    
    // Subscribe to joint states to monitor if we're receiving data
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 
        rclcpp::QoS(10).reliable(), 
        std::bind(&MoveitInterface::jointStateCallback, this, std::placeholders::_1)
    );
    
    // Add node to executor and start spinning in a separate thread
    executor_->add_node(moveit_node_);
    
    // Initialize MoveGroupInterface with the dedicated node
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
    move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, 
        "arm_group"
    );
    
    // Set the end effector link
    move_group_ptr_->setEndEffectorLink(end_effector_link_);
    
    // Configure the planner with more robust settings
    move_group_ptr_->setPlanningPipelineId("ompl");
    move_group_ptr_->setPlanningTime(20.0);
    move_group_ptr_->setMaxVelocityScalingFactor(0.3);
    move_group_ptr_->setMaxAccelerationScalingFactor(0.3);
    move_group_ptr_->setGoalPositionTolerance(0.02);
    move_group_ptr_->setGoalOrientationTolerance(0.02);
    move_group_ptr_->setNumPlanningAttempts(5);  // Try multiple planning attempts
    move_group_ptr_->allowReplanning(true);      // Allow replanning if needed
    move_group_ptr_->startStateMonitor();
    
    // Start spinning in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    
    RCLCPP_INFO(this->get_logger(), "MoveitInterface initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using '%s' as end effector link", end_effector_link_.c_str());
}

// Callback for joint states
void MoveitInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received joint state with timestamp sec=%d, nanosec=%d", 
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    received_joint_state_ = true;
}

// Wait for robot state to be available
void MoveitInterface::waitForRobotState(double timeout_seconds) {
    RCLCPP_INFO(this->get_logger(), "Waiting for robot state data...");
    auto start = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        if (received_joint_state_) {
            RCLCPP_INFO(this->get_logger(), "Robot state data received!");
            return;
        }
        
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > timeout_seconds) {
            RCLCPP_WARN(this->get_logger(), 
                       "Timeout waiting for robot state after %.1f seconds", timeout_seconds);
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
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

// Implement method to plan and execute movement with more robustness
bool MoveitInterface::planAndExecuteMovement(const geometry_msgs::msg::Pose& target_pose)
{
    if (!received_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet. Movement may fail.");
    }
    
    // Set planning parameters for this move
    move_group_ptr_->setGoalTolerance(0.01); // Set the tolerance to 1 cm
    
    // Try to set the target with both exact and approximate methods
    bool target_set = false;
    
    try {
        RCLCPP_INFO(this->get_logger(), "Attempting to set pose target...");
        move_group_ptr_->setPoseTarget(target_pose, end_effector_link_);
        target_set = true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to set exact pose target: %s", e.what());
        RCLCPP_INFO(this->get_logger(), "Trying approximate IK solution instead...");
        
        try {
            move_group_ptr_->setApproximateJointValueTarget(target_pose, end_effector_link_);
            target_set = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set approximate pose target: %s", e.what());
            return false;
        }
    }
    
    if (!target_set) {
        RCLCPP_ERROR(this->get_logger(), "Could not set target pose!");
        return false;
    }

    // Create a plan with multiple attempts
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = false;
    int max_attempts = 3;
    
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        RCLCPP_INFO(this->get_logger(), "Planning attempt %d/%d...", attempt, max_attempts);
        success = static_cast<bool>(move_group_ptr_->plan(plan));
        
        if (success) {
            break;
        } else if (attempt < max_attempts) {
            RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed, retrying...", attempt);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning successful! Executing plan...");
        
        try {
            moveit::planning_interface::MoveItErrorCode execute_result = move_group_ptr_->execute(plan);
            if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Execution complete!");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", execute_result.val);
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

// Implement method to get current pose with retry logic
geometry_msgs::msg::PoseStamped MoveitInterface::getCurrentPose(int max_retries)
{
    for (int i = 0; i < max_retries; i++) {
        try {
            auto pose = move_group_ptr_->getCurrentPose(end_effector_link_);
            return pose;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get current pose (attempt %d/%d): %s", 
                       i+1, max_retries, e.what());
            
            if (i < max_retries - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } else {
                throw; // Re-throw on last attempt
            }
        }
    }
    
    // This line shouldn't be reached due to the throw above, but just in case
    throw std::runtime_error("Failed to get current pose after multiple attempts");
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
    try {
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
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to display joint limits: %s", e.what());
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
            
            csv_file.flush();  // Make sure data is written to file
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
    
    try {
        // Create the MoveitInterface instance
        auto moveit_interface = std::make_shared<MoveitInterface>(node_options);
        
        // Wait for robot state data to be available
        moveit_interface->waitForRobotState(10.0);  // Wait up to 10 seconds
        
        // Display debugging information
        moveit_interface->displayAvailableNamedStates();
        
        // Only proceed with these operations if we have valid robot state
        try {
            moveit_interface->displayPlanningSceneBoundingBox();
            moveit_interface->displayJointLimits();
        } catch (const std::exception& e) {
            RCLCPP_WARN(moveit_interface->get_logger(), 
                       "Could not display all debugging info: %s", e.what());
        }
        
        // Try to print the current pose to verify access to robot state
        try {
            RCLCPP_INFO(moveit_interface->get_logger(), "Attempting to get current pose...");
            auto current_pose = moveit_interface->getCurrentPose();
            RCLCPP_INFO(moveit_interface->get_logger(), 
                       "Current position: x=%f, y=%f, z=%f", 
                       current_pose.pose.position.x,
                       current_pose.pose.position.y,
                       current_pose.pose.position.z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(moveit_interface->get_logger(), 
                        "Exception while getting robot state: %s", e.what());
            RCLCPP_INFO(moveit_interface->get_logger(), 
                       "Will try to continue with a simple move to a named state instead...");
            
            // Try moving to a named state as a fallback
            if (moveit_interface->moveToNamedState("zero_pose")) {
                RCLCPP_INFO(moveit_interface->get_logger(), "Successfully moved to zero_pose");
            } else {
                RCLCPP_ERROR(moveit_interface->get_logger(), "Failed to move to zero_pose");
            }
            
            rclcpp::sleep_for(std::chrono::seconds(5));
            rclcpp::shutdown();
            return 0;
        }
        
        // Define the target pose directly
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = -0.509;  // X position
        target_pose.position.y = -0.247;  // Y position
        target_pose.position.z = 0.543;   // Z position
        
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
            // Try a named state as fallback
            RCLCPP_INFO(moveit_interface->get_logger(), "Trying to move to a named state as fallback...");
            moveit_interface->moveToNamedState("pick");
        } else {
            RCLCPP_INFO(moveit_interface->get_logger(), "Successfully moved to the target pose.");
        }
        
        // Keep the node alive for 5 seconds
        rclcpp::sleep_for(std::chrono::seconds(5));
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
    }
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
