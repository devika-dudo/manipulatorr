#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <cmath>
#include <iomanip>

std::atomic<bool> logging_active(true);

double round_to_decimal(double value, int decimal_places) {
    double factor = std::pow(10, decimal_places);
    return std::round(value * factor) / factor;
}

void log_tool0_position(rclcpp::Node::SharedPtr node, 
                        moveit::planning_interface::MoveGroupInterface& move_group_interface) {
    std::ofstream csv_file("tool0_position_log.csv");
    csv_file << "Time;X;Y;Z\n";
    csv_file << std::fixed << std::setprecision(4);
    
    while (logging_active) {
        try {
            auto current_pose = move_group_interface.getCurrentPose("tool0");
            auto now = node->get_clock()->now();
            csv_file << now.seconds() << ";"
                     << round_to_decimal(current_pose.pose.position.x * 1000, 4) << ";"
                     << round_to_decimal(current_pose.pose.position.y * 1000, 4) << ";"
                     << round_to_decimal(current_pose.pose.position.z * 1000, 4) << "\n";
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "Could not get current pose: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Log every 100 ms
    }
    csv_file.close();
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    // Create a logger
    // At the beginning of your main function, after creating the node

    auto const logger = rclcpp::get_logger("hello_moveit");
    
    // Give the system time to establish connections
    RCLCPP_INFO(logger, "Waiting for connections to establish...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Create the MoveIt MoveGroup Interface
    RCLCPP_INFO(logger, "Creating MoveGroupInterface...");
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm_group");
    
    // Set up the planning pipeline and planner
    move_group_interface.setPlanningPipelineId("ompl");
    move_group_interface.setPlannerId("RRTConnectkConfigDefault");
    move_group_interface.setPlanningTime(5.0);  // Increased planning time
    move_group_interface.setMaxVelocityScalingFactor(0.5);  // Slower for safety
    move_group_interface.setMaxAccelerationScalingFactor(0.5);  // Slower for safety
    
    // Set goal tolerances
    move_group_interface.setGoalPositionTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);
    
    // Wait for the robot model to be available
    RCLCPP_INFO(logger, "Waiting for robot model to be loaded...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Try to print the current pose to see if we can access the robot state
    try {
        auto current_state = move_group_interface.getCurrentState();
        if (current_state) {
            RCLCPP_INFO(logger, "Successfully got current robot state");
        } else {
            RCLCPP_ERROR(logger, "Failed to get current robot state");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception while getting robot state: %s", e.what());
    }
    
    // Define the target pose for the end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.537123;  // Set your desired X position
    target_pose.position.y = -0.512234; // Set your desired Y position
    target_pose.position.z = 0.631764;  // Set your desired Z position
    
    // Use a properly normalized quaternion
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    
    RCLCPP_INFO(logger, "Setting target pose...");
    move_group_interface.setPoseTarget(target_pose);
    
    // Start logging tool0 positions in a separate thread after initial setup
    RCLCPP_INFO(logger, "Starting position logging...");
    std::thread logging_thread(log_tool0_position, node, std::ref(move_group_interface));
    
    // Create a plan to the target pose
    RCLCPP_INFO(logger, "Planning trajectory...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(move_group_interface.plan(plan));
    
    // Try to execute the movement plan if it was created successfully
    if (success) {
        RCLCPP_INFO(logger, "Planning successful! Executing plan...");
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution complete!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    
    // Stop logging when movements are done
    logging_active = false;
    if (logging_thread.joinable()) {
        logging_thread.join(); // Wait for logging to finish
    }
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
