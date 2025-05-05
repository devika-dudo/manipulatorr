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
    
    // Method to wait for joint states with timeout
    bool waitForJointStates(const std::chrono::seconds timeout = std::chrono::seconds(5));
    
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
    
    // Store the latest joint state
    sensor_msgs::msg::JointState latest_joint_state_;
    
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
    
    // Subscribe to joint states on both nodes with reliable QoS
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", // Use absolute topic name
        rclcpp::QoS(10).reliable(), // Use reliable QoS
        std::bind(&MoveitInterface::jointStateCallback, this, std::placeholders::_1)
    );
    
    // Add ONLY the MoveIt node to executor
    executor_->add_node(moveit_node_);
    
    // Start spinning in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    
    // Give a moment for the executor to start
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Initialize MoveGroupInterface with the dedicated node
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
    try {
        move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node_, 
            "arm_group"
        );
        
        // Set the end effector link
        move_group_ptr_->setEndEffectorLink(end_effector_link_);
        
        // Configure the planner with more robust settings
        move_group_ptr_->setPlanningPipelineId("ompl");
        // Set the planner to RRT*
        move_group_ptr_->setPlannerId("RRTstar");
        move_group_ptr_->setPlanningTime(20.0);
        move_group_ptr_->setMaxVelocityScalingFactor(0.3);
        move_group_ptr_->setMaxAccelerationScalingFactor(0.3);
        move_group_ptr_->setGoalPositionTolerance(0.02);
        move_group_ptr_->setGoalOrientationTolerance(0.02);
        move_group_ptr_->setNumPlanningAttempts(5);  // Try multiple planning attempts
        move_group_ptr_->allowReplanning(true);      // Allow replanning if needed
        move_group_ptr_->startStateMonitor();
        
        RCLCPP_INFO(this->get_logger(), "MoveitInterface initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Using '%s' as end effector link", end_effector_link_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
        throw;
    }
}

// Callback for joint states
void MoveitInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Store the latest joint state
    latest_joint_state_ = *msg;
    
    if (!received_joint_state_) {
        RCLCPP_INFO(this->get_logger(), "Received first joint state with %zu joints", msg->name.size());
        received_joint_state_ = true;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Received joint state with %zu joints", msg->name.size());
    }
}

// Wait for joint states with timeout
bool MoveitInterface::waitForJointStates(const std::chrono::seconds timeout) {
    auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // 10Hz checking
    
    while (!received_joint_state_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Waiting for joint states...");
        
        // Check for timeout
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for joint states after %d seconds!", 
                      static_cast<int>(timeout.count()));
            return false;
        }
        
        // Process any pending callbacks
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
    
    RCLCPP_INFO(this->get_logger(), "Joint states received successfully, proceeding with motion planning");
    return true;
}

// Wait for robot state to be available
void MoveitInterface::waitForRobotState(double timeout_seconds) {
    RCLCPP_INFO(this->get_logger(), "Waiting for robot state data...");
    auto start = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        if (received_joint_state_) {
            RCLCPP_INFO(this->get_logger(), "Robot state data received!");
            
            // Double-check that MoveGroupInterface can access the robot state
            try {
                auto robot_state = move_group_ptr_->getCurrentState();
                if (robot_state) {
                    RCLCPP_INFO(this->get_logger(), "MoveIt has access to robot state");
                    return;
                } else {
                    RCLCPP_WARN(this->get_logger(), "MoveIt cannot access robot state yet");
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error checking MoveIt robot state: %s", e.what());
            }
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

// Implement method to plan and execute movement with improved joint state handling
bool MoveitInterface::planAndExecuteMovement(const geometry_msgs::msg::Pose& target_pose)
{
    // Wait for joint states before attempting movement
    if (!received_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet. Waiting for joint states...");
        if (!waitForJointStates(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint states. Continuing anyway but movement may fail.");
            // We'll continue anyway and let MoveIt try to work with what it has
        }
    }
    
    // Reset any previous targets
    move_group_ptr_->clearPoseTargets();
    
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
        
        try {
            auto plan_result = move_group_ptr_->plan(plan);
            success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Planning successful!");
                break;
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning attempt %d failed with code: %d", 
                           attempt, plan_result.val);
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
        RCLCPP_INFO(this->get_logger(), "Planning successful! Executing plan...");
        
        try {
            moveit::core::MoveItErrorCode execute_result = move_group_ptr_->execute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Execution complete!");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", execute_result.val);
                // Try direct execution as a fallback
                RCLCPP_INFO(this->get_logger(), "Trying direct async execution...");
                execute_result = move_group_ptr_->asyncExecute(plan);
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Async execution started!");
                    // Wait for the movement to complete in ROS2
                    std::this_thread::sleep_for(std::chrono::seconds(5));  // Simple timeout approach
                    RCLCPP_INFO(this->get_logger(), "Async execution complete!");
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Async execution also failed with code: %d", execute_result.val);
                    return false;
                }
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

// Implement method to move to a named state
bool MoveitInterface::moveToNamedState(const std::string& state_name)
{
    // First, make sure we have joint states
    if (!received_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet. Waiting for joint states...");
        if (!waitForJointStates(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint states. Continuing anyway but movement may fail.");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Moving to named state: %s", state_name.c_str());
    
    try {
        move_group_ptr_->setNamedTarget(state_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set named target '%s': %s", 
                    state_name.c_str(), e.what());
        return false;
    }
    
    // Create a plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    
    try {
        auto plan_result = move_group_ptr_->plan(plan);
        success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Planning to %s failed with code: %d", 
                        state_name.c_str(), plan_result.val);
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning to %s: %s", 
                    state_name.c_str(), e.what());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning to %s successful! Executing plan...", state_name.c_str());
    
    try {
        auto execute_result = move_group_ptr_->execute(plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Execution to %s complete!", state_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Execution to %s failed with code: %d", 
                        state_name.c_str(), execute_result.val);
            
            // Try async execution as fallback
            RCLCPP_INFO(this->get_logger(), "Trying async execution to %s...", state_name.c_str());
            execute_result = move_group_ptr_->asyncExecute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                // Wait for the movement to complete with a fixed timeout
                std::this_thread::sleep_for(std::chrono::seconds(5));  // Simple timeout approach
                RCLCPP_INFO(this->get_logger(), "Async execution to %s complete!", state_name.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Async execution to %s also failed with code: %d", 
                            state_name.c_str(), execute_result.val);
                return false;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception during execution to %s: %s", 
                    state_name.c_str(), e.what());
        return false;
    }
}

// Implement method to get current pose with retry logic
geometry_msgs::msg::PoseStamped MoveitInterface::getCurrentPose(int max_retries)
{
    for (int i = 0; i < max_retries; i++) {
        try {
            auto pose = move_group_ptr_->getCurrentPose(end_effector_link_);
            // Verify that the pose is valid
            if (std::isfinite(pose.pose.position.x) && 
                std::isfinite(pose.pose.position.y) && 
                std::isfinite(pose.pose.position.z)) {
                return pose;
            } else {
                RCLCPP_WARN(this->get_logger(), "Got invalid pose with NaN values");
                if (i < max_retries - 1) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                } else {
                    throw std::runtime_error("Received invalid pose with NaN values");
                }
            }
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
    try {
        auto available_states = move_group_ptr_->getNamedTargets();
        RCLCPP_INFO(this->get_logger(), "Available named states:");
        if (available_states.empty()) {
            RCLCPP_INFO(this->get_logger(), "  No named states available");
        } else {
            for (const auto& state : available_states) {
                RCLCPP_INFO(this->get_logger(), "  - %s", state.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get named targets: %s", e.what());
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
        
        // Wait for joint states with a shorter timeout first to see if they're coming quickly
        if (!moveit_interface->waitForJointStates(std::chrono::seconds(2))) {
            RCLCPP_WARN(moveit_interface->get_logger(), 
                       "Initial joint state wait timed out. Will continue anyway.");
            
            // Check the joint state topic to verify it exists
            RCLCPP_INFO(moveit_interface->get_logger(), 
                      "Checking if joint state topic exists. Run 'ros2 topic list' in another terminal to verify.");
        }
        
        // Wait for robot state data to be available with a longer timeout
        moveit_interface->waitForRobotState(10.0);  // Wait up to 10 seconds
        
        // Display debugging information
        try {
            moveit_interface->displayAvailableNamedStates();
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
                RCLCPP_INFO(moveit_interface->get_logger(), "Will try alternate named states...");
                
                // Try some common alternative named state names
                const std::vector<std::string> fallback_states = {
                    "home", "ready", "default", "start", "initial"
                };
                
                for (const auto& state : fallback_states) {
                    RCLCPP_INFO(moveit_interface->get_logger(), "Trying to move to '%s' state...", state.c_str());
                    if (moveit_interface->moveToNamedState(state)) {
                        RCLCPP_INFO(moveit_interface->get_logger(), "Successfully moved to %s state", state.c_str());
                        break;
                    }
                }
            }
            
            rclcpp::sleep_for(std::chrono::seconds(5));
            rclcpp::shutdown();
            return 0;
        }
        
        // Define the target pose directly
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = -0.167;  // X position
        target_pose.position.y = -0.047;  // Y position
        target_pose.position.z = 0.840;   // Z position

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
