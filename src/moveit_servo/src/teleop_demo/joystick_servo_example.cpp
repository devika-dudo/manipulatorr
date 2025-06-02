/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 *      Modified  : For Logitech Extreme 3D Pro compatibility with deadband
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <cmath>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "link_5";
const std::string BASE_FRAME_ID = "base_link";

// Deadband configuration
const double STICK_DEADBAND = 0.1;      // Deadband for main stick axes (10% of full range)
const double TWIST_DEADBAND = 0.15;     // Deadband for stick twist (15% of full range)
const double HAT_DEADBAND = 0.05;       // Deadband for hat switch (smaller since it's more precise)
const double THROTTLE_DEADBAND = 0.05;  // Deadband for throttle

// Enums for button names -> axis/button array index
// For Logitech Extreme 3D Pro
enum Axis
{
  STICK_X = 0,        // Main stick left/right
  STICK_Y = 1,        // Main stick forward/back
  STICK_TWIST = 2,    // Stick rotation (twist)
  THROTTLE = 3,       // Throttle slider
  HAT_X = 4,          // Hat switch left/right
  HAT_Y = 5           // Hat switch up/down
};

enum Button
{
  TRIGGER = 0,        // Main trigger
  BUTTON_2 = 1,       // Thumb button
  BUTTON_3 = 2,       // Button 3 on base
  BUTTON_4 = 3,       // Button 4 on base
  BUTTON_5 = 4,       // Button 5 on base
  BUTTON_6 = 5,       // Button 6 on base
  BUTTON_7 = 6,       // Button 7 on base
  BUTTON_8 = 7,       // Button 8 on base
  BUTTON_9 = 8,       // Button 9 on base
  BUTTON_10 = 9,      // Button 10 on base
  BUTTON_11 = 10,     // Button 11 on base
  BUTTON_12 = 11      // Button 12 on base
};

// Logitech Extreme 3D Pro has throttle with default at -1.0, hat switches default at 0.0
std::map<Axis, double> AXIS_DEFAULTS = { { THROTTLE, -1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// Control mapping for Logitech Extreme 3D Pro:
// - Main stick X/Y: End-effector linear X/Y movement
// - Stick twist: End-effector rotation around Z
// - Throttle: End-effector linear Z movement
// - Hat switch: End-effector rotation around X/Y
// - Buttons 3-6: Joint jogging for joints 1,2,6,7
// - Trigger: Frame switching
// - Button 2: Emergency stop or special function

/** \brief Apply deadband to an axis value
 * @param value The raw axis value
 * @param deadband The deadband threshold (0.0 to 1.0)
 * @return The processed value with deadband applied
 */
double applyDeadband(double value, double deadband)
{
  double abs_value = std::abs(value);
  
  // If within deadband, return 0
  if (abs_value < deadband)
  {
    return 0.0;
  }
  
  // Scale the remaining range to full output
  // This ensures smooth transition from deadband edge to full range
  double sign = (value > 0) ? 1.0 : -1.0;
  double scaled_value = (abs_value - deadband) / (1.0 - deadband);
  
  return sign * scaled_value;
}

/** \brief This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority - Check ALL joint control buttons (3-12)
  // If any joint jog command is requested, we are only publishing joint commands
  if (buttons[BUTTON_3] || buttons[BUTTON_4] || buttons[BUTTON_5] || buttons[BUTTON_6] ||
      buttons[BUTTON_7] || buttons[BUTTON_8] || buttons[BUTTON_9] || buttons[BUTTON_10] ||
      buttons[BUTTON_11] || buttons[BUTTON_12])
  {
    // Map buttons to joint movements
    joint->joint_names.push_back("joint_1");
    joint->velocities.push_back(buttons[BUTTON_3] - buttons[BUTTON_4]); // Button 3 positive, Button 4 negative
    
    joint->joint_names.push_back("joint_2");
    joint->velocities.push_back(buttons[BUTTON_5] - buttons[BUTTON_6]); // Button 5 positive, Button 6 negative
    
    // Use buttons 7,8 for joint_5 and buttons 9,10 for joint_6
    if (buttons[BUTTON_7] || buttons[BUTTON_8])
    {
      joint->joint_names.push_back("joint_5");
      joint->velocities.push_back(buttons[BUTTON_7] - buttons[BUTTON_8]);
    }
    
    if (buttons[BUTTON_9] || buttons[BUTTON_10])
    {
      joint->joint_names.push_back("joint_6");
      joint->velocities.push_back(buttons[BUTTON_9] - buttons[BUTTON_10]);
    }
    
    // Add joint_4 control using buttons 11,12 if you have joint_4
    if (buttons[BUTTON_11] || buttons[BUTTON_12])
    {
      joint->joint_names.push_back("joint_4");
      joint->velocities.push_back(buttons[BUTTON_11] - buttons[BUTTON_12]);
    }
    
    return false;
  }

  // Apply deadband to all axis values before processing
  double stick_x = applyDeadband(axes[STICK_X], STICK_DEADBAND);
  double stick_y = applyDeadband(axes[STICK_Y], STICK_DEADBAND);
  double stick_twist = applyDeadband(axes[STICK_TWIST], TWIST_DEADBAND);
  double hat_x = applyDeadband(axes[HAT_X], HAT_DEADBAND);
  double hat_y = applyDeadband(axes[HAT_Y], HAT_DEADBAND);
  
  // Process throttle with its own deadband (accounting for default offset)
  double throttle_raw = axes[THROTTLE];
  double throttle_normalized = (throttle_raw - AXIS_DEFAULTS.at(THROTTLE)) / 2.0; // Convert to [0,1] range
  double throttle_deadbanded = applyDeadband(throttle_normalized - 0.5, THROTTLE_DEADBAND); // Center around 0 and apply deadband

  // The main control: map joystick to twist commands with deadband applied
  // Main stick controls linear X and Y movement
  twist->twist.linear.x = stick_y;        // Forward/back
  twist->twist.linear.y = -stick_x;       // Left/right (inverted for intuitive control)
  
  // Throttle controls linear Z movement (up/down)
  twist->twist.linear.z = throttle_deadbanded;
  
  // Stick twist controls rotation around Z axis (roll)
  twist->twist.angular.z = stick_twist;
  
  // Hat switch controls rotation around X and Y axes
  twist->twist.angular.x = hat_y;         // Pitch
  twist->twist.angular.y = -hat_x;        // Yaw (inverted for intuitive control)

  return true;
}

/** \brief This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  // Use trigger to switch between end-effector frame and base frame
  static bool trigger_pressed_last = false;
  bool trigger_pressed_now = buttons[TRIGGER];
  
  // Toggle frame on trigger press (not hold)
  if (trigger_pressed_now && !trigger_pressed_last)
  {
    if (frame_name == EEF_FRAME_ID)
      frame_name = BASE_FRAME_ID;
    else
      frame_name = EEF_FRAME_ID;
  }
  
  trigger_pressed_last = trigger_pressed_now;
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    collision_pub_ =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // Collision publisher setup (not used for tables anymore)
    // collision_pub_thread_ = std::thread([this]() {
    //   // Table collision objects removed per user request
    // });

    // Print control instructions
    RCLCPP_INFO(this->get_logger(), "Logitech Extreme 3D Pro Controls (with deadband):");
    RCLCPP_INFO(this->get_logger(), "  Main Stick: Linear X/Y movement (deadband: %.1f%%)", STICK_DEADBAND * 100);
    RCLCPP_INFO(this->get_logger(), "  Stick Twist: Rotation around Z (deadband: %.1f%%)", TWIST_DEADBAND * 100);
    RCLCPP_INFO(this->get_logger(), "  Throttle: Linear Z movement (deadband: %.1f%%)", THROTTLE_DEADBAND * 100);
    RCLCPP_INFO(this->get_logger(), "  Hat Switch: Rotation around X/Y (deadband: %.1f%%)", HAT_DEADBAND * 100);
    RCLCPP_INFO(this->get_logger(), "  Trigger: Toggle control frame");
    RCLCPP_INFO(this->get_logger(), "  Buttons 3-6: Joint control (pairs: 3/4 for joint1, 5/6 for joint2)");
    RCLCPP_INFO(this->get_logger(), "  Buttons 7-10: Additional joint control (7/8 for joint_5, 9/10 for joint_6)");
  }

  ~JoyToServoPub() override
  {
    // Thread cleanup no longer needed since collision thread is disabled
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "link_3";
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;

  std::thread collision_pub_thread_;
};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
