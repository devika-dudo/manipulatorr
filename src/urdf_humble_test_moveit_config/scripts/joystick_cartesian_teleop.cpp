#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class JoystickCartesianTeleop : public rclcpp::Node
{
public:
    JoystickCartesianTeleop()
        : Node("joystick_cartesian_teleop")
    {
        // Declare parameters first
        this->declare_parameter("step", 0.01);  // 1 cm
        this->declare_parameter("rot_step", 0.05); // ~3 deg
        step_ = this->get_parameter("step").as_double();
        rot_step_ = this->get_parameter("rot_step").as_double();

        // Create subscription
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoystickCartesianTeleop::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Joystick Cartesian Teleop Node Started. Example:\nros2 topic pub /joy sensor_msgs/msg/Joy \"{axes: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: []}\" -r 10");
        
        // Initialize MoveGroupInterface after node is fully constructed
        initializeMoveGroup();
    }

private:
    void initializeMoveGroup()
    {
        try {
            // Use this approach instead of shared_from_this() in constructor
            move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
                "arm_group"
            );
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!move_group_) {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized");
            return;
        }

        try {
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
            geometry_msgs::msg::Pose target_pose = current_pose.pose;

            if (msg->axes.size() >= 6) {
                // Update position
                target_pose.position.x += msg->axes[0] * step_; // Left/Right
                target_pose.position.y += msg->axes[1] * step_; // Forward/Backward
                target_pose.position.z += msg->axes[2] * step_; // Up/Down

                // Update orientation
                tf2::Quaternion q_orig, q_rot, q_new;
                tf2::fromMsg(current_pose.pose.orientation, q_orig);

                tf2::Quaternion roll_q, pitch_q, yaw_q;
                roll_q.setRPY(msg->axes[3] * rot_step_, 0, 0);
                pitch_q.setRPY(0, msg->axes[4] * rot_step_, 0);
                yaw_q.setRPY(0, 0, msg->axes[5] * rot_step_);

                q_rot = yaw_q * pitch_q * roll_q;
                q_new = q_rot * q_orig;
                q_new.normalize();

                target_pose.orientation = tf2::toMsg(q_new);
            }

            // Plan and execute cartesian path
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.005;
            const double jump_threshold = 0.0;

            double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.0) {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group_->execute(plan);
            } else {
                RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in joyCallback: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    double step_;
    double rot_step_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickCartesianTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
