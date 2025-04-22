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
    
    // Method to wait for robot state to be available
    void waitForRobotState(double timeout_seconds = 10.0);
    
    // Getter for move_group_ptr_
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup() {
        return move_group_ptr_;
    }
    
    // Getter for moveit_node_
    rclcpp::Node::SharedPtr getMoveitNode() {
        return moveit_node_;
    }
    
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
    : Node("plan_around_objects", options),
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
    
    RCLCPP_INFO(this->get_logger(), "Joint states received successfully");
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

// Implement method to plan and execute movement
bool MoveitInterface::planAndExecuteMovement(const geometry_msgs::msg::Pose& target_pose)
{
    // Wait for joint states before attempting movement
    if (!received_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet. Waiting for joint states...");
        if (!waitForJointStates(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint states. Continuing anyway but movement may fail.");
        }
    }
    
    // Reset any previous targets
    move_group_ptr_->clearPoseTargets();
    
    // Set planning parameters for this move
    move_group_ptr_->setGoalTolerance(0.01); // Set the tolerance to 1 cm
    
    // Try to set the target with both exact and approximate methods
    bool target_set = false;
    
    try {
        RCLCPP_INFO(this->get_logger(), "Setting pose target...");
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
                    std::this_thread::sleep_for(std::chrono::seconds(5));
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
                std::this_thread::sleep_for(std::chrono::seconds(5));
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

// Helper function to create a collision object
moveit_msgs::msg::CollisionObject createCollisionBox(
    const std::string& frame_id,
    const std::string& id,
    double x, double y, double z, 
    double width, double depth, double height,
    const rclcpp::Logger& logger)
{
    moveit_msgs::msg::CollisionObject collision_object;
    
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = width;
    primitive.dimensions[primitive.BOX_Y] = depth;
    primitive.dimensions[primitive.BOX_Z] = height;
    
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;
    box_pose.orientation.w = 1.0;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    RCLCPP_INFO(logger, "Created collision object: %s at position (%.2f, %.2f, %.2f)", 
                id.c_str(), x, y, z);
    
    return collision_object;
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
        auto logger = moveit_interface->get_logger();
        
        // Wait for joint states and robot state
        moveit_interface->waitForJointStates(std::chrono::seconds(2));
        moveit_interface->waitForRobotState(5.0);
        
        // Try to print the current pose
        try {
            auto current_pose = moveit_interface->getCurrentPose();
            RCLCPP_INFO(logger, "Current position: x=%.3f, y=%.3f, z=%.3f", 
                      current_pose.pose.position.x,
                      current_pose.pose.position.y,
                      current_pose.pose.position.z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Exception while getting robot state: %s", e.what());
            
            // Try moving to a named state as a fallback
            moveit_interface->moveToNamedState("zero_pose");
            rclcpp::shutdown();
            return 0;
        }
        
        // Set up MoveIt Visual Tools
        auto move_group_ptr = moveit_interface->getMoveGroup();
        auto robot_model = move_group_ptr->getRobotModel();

        // Create the visual tools with the proper node
        auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            moveit_interface->getMoveitNode(),   // Use the proper node
            "base_link",                         // Base frame
            "visual_tools",                      // Topic for markers
            robot_model                          // Robot model
        );
        
        // Initialize visual tools
        visual_tools->deleteAllMarkers();
        visual_tools->loadRemoteControl();
        
        // Define text display lambda
        auto draw_title = [&visual_tools](const std::string& text) {
            Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
            text_pose.translation().z() = 1.0;
            visual_tools->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        };
        
        // Lambda for prompts
        auto prompt = [&visual_tools](const std::string& text) {
            visual_tools->prompt(text);
        };
        
        // Lambda for trajectory visualization
        auto draw_trajectory_tool_path = [&visual_tools, &move_group_ptr](const moveit_msgs::msg::RobotTrajectory& trajectory) {
            visual_tools->publishTrajectoryLine(
                trajectory, 
                move_group_ptr->getRobotModel()->getJointModelGroup("arm_group")
            );
        };
        
        // Create collision object
        auto collision_object = createCollisionBox(
            move_group_ptr->getPlanningFrame(),  // Frame ID
            "box1",                              // Object ID
            0.22, 0.0, 0.25,                     // Position x, y, z
            0.2, 0.05, 0.50,                     // Dimensions width, depth, height
            logger
        );
        
        // Add the collision object to the planning scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObject(collision_object);
        
        // Define the target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = -0.509;
        target_pose.position.y = -0.247;
        target_pose.position.z = 0.543;
        target_pose.orientation.x = -0.600;
        target_pose.orientation.y = -0.517;
        target_pose.orientation.z = 0.093;
        target_pose.orientation.w = 0.604;
        
        // Log the target pose
        RCLCPP_INFO(logger, "Planning to target: x=%.3f, y=%.3f, z=%.3f", 
                   target_pose.position.x,
                   target_pose.position.y,
                   target_pose.position.z);
                   
        // User prompt for planning
        prompt("Press 'next' in the RvizVisualToolsGui window to plan");
        draw_title("Planning");
        visual_tools->trigger();
        
        // Plan motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_ptr->setPoseTarget(target_pose);
        auto plan_result = move_group_ptr->plan(plan);
        bool success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
        
        // Execute if planning succeeded
        if (success) {
            draw_trajectory_tool_path(plan.trajectory_);  // Note the underscore here
            visual_tools->trigger();
            
            prompt("Press 'next' in the RvizVisualToolsGui window to execute");
            draw_title("Executing");
            visual_tools->trigger();
            
            // Execute the plan
            move_group_ptr->execute(plan);
            RCLCPP_INFO(logger, "Successfully moved to the target pose.");
        } else {
            draw_title("Planning Failed!");
            visual_tools->trigger();
            RCLCPP_ERROR(logger, "Planning failed!");
        }
        
        // Keep the node alive for a moment before shutting down
        rclcpp::sleep_for(std::chrono::seconds(3));
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
    }
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
