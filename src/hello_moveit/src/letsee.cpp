#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_one_waypoint");

    // Create MoveGroupInterface for the manipulator
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm_group");

    // Optional: set planner, scaling, time
    move_group.setPlanningTime(3.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // Define a target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.537123;
    target_pose.position.y = -0.512234;
    target_pose.position.z = 0.631764;
    target_pose.orientation.w = 1.0;  // Identity quaternion

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and move
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}


