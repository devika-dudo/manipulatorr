#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <vector>

class CartesianPlanner : public rclcpp::Node
{
public:
  CartesianPlanner() : Node("cartesian_planner")
  {
    // Declare parameter for planning group
    this->declare_parameter<std::string>("planning_group", "arm");
    std::string planning_group = this->get_parameter("planning_group").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interface for group: %s", planning_group.c_str());
    
    // Initialize MoveIt interface - use rclcpp::Node::SharedPtr instead of shared_from_this()
    arm_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), planning_group);
    
    // Set planning parameters
    arm_group_interface_->setPlanningTime(10.0);
    arm_group_interface_->setNumPlanningAttempts(5);
    arm_group_interface_->setMaxVelocityScalingFactor(0.3); // Slower for scanning
    arm_group_interface_->setMaxAccelerationScalingFactor(0.3);
    
    // FIXED: Use correct OMPL planner ID format
    arm_group_interface_->setPlannerId("RRTConnect");
    
    // Alternative planner options you can try:
    // arm_group_interface_->setPlannerId("RRT");
    // arm_group_interface_->setPlannerId("PRM");
    // arm_group_interface_->setPlannerId("BiTRRT");
    
    // ADDED: Set planning pipeline explicitly to OMPL
    arm_group_interface_->setPlanningPipelineId("ompl");
    
    // Subscribers for different types of movement
    single_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/move_to_pose", 10,
        std::bind(&CartesianPlanner::singlePoseCallback, this, std::placeholders::_1));
    
    cartesian_path_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/cartesian_path", 10,
        std::bind(&CartesianPlanner::cartesianPathCallback, this, std::placeholders::_1));
    
    sweep_horizontal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/sweep_horizontal", 10,
        std::bind(&CartesianPlanner::sweepHorizontalCallback, this, std::placeholders::_1));
    
    sweep_vertical_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/sweep_vertical", 10,
        std::bind(&CartesianPlanner::sweepVerticalCallback, this, std::placeholders::_1));
    
    grid_scan_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/grid_scan", 10,
        std::bind(&CartesianPlanner::gridScanCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Cartesian Planner Node Initialized");
    RCLCPP_INFO(this->get_logger(), "Planning Pipeline: %s", arm_group_interface_->getPlanningPipelineId().c_str());
    RCLCPP_INFO(this->get_logger(), "Planner ID: %s", arm_group_interface_->getPlannerId().c_str());
    RCLCPP_INFO(this->get_logger(), "Available topics:");
    RCLCPP_INFO(this->get_logger(), "  /move_to_pose - Single pose movement [x,y,z,r,p,y]");
    RCLCPP_INFO(this->get_logger(), "  /cartesian_path - Linear path [x1,y1,z1,r,p,y,x2,y2,z2,steps]");
    RCLCPP_INFO(this->get_logger(), "  /sweep_horizontal - [start_x,start_y,start_z,r,p,y,distance,steps]");
    RCLCPP_INFO(this->get_logger(), "  /sweep_vertical - [start_x,start_y,start_z,r,p,y,distance,steps]");
    RCLCPP_INFO(this->get_logger(), "  /grid_scan - [center_x,center_y,center_z,r,p,y,width,height,rows,cols]");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_interface_;
  std::mutex planning_mutex_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr single_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sweep_horizontal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sweep_vertical_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grid_scan_sub_;

  // Your existing single pose movement
  void singlePoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "Expected 6 values [x,y,z,roll,pitch,yaw], got %zu", 
                   msg->data.size());
      return;
    }
    
    geometry_msgs::msg::Pose target_pose = createPoseFromArray(msg->data, 0);
    executeSinglePose(target_pose);
  }

  // Cartesian path between two points
  void cartesianPathCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 10) {
      RCLCPP_ERROR(this->get_logger(), "Expected 10 values [x1,y1,z1,r,p,y,x2,y2,z2,steps], got %zu", 
                   msg->data.size());
      return;
    }
    
    geometry_msgs::msg::Pose start_pose = createPoseFromArray(msg->data, 0);
    geometry_msgs::msg::Pose end_pose;
    end_pose.position.x = msg->data[6];
    end_pose.position.y = msg->data[7];
    end_pose.position.z = msg->data[8];
    end_pose.orientation = start_pose.orientation; // Same orientation
    
    int steps = static_cast<int>(msg->data[9]);
    
    executeCartesianPath(start_pose, end_pose, steps);
  }

  // Horizontal sweep
  void sweepHorizontalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 8) {
      RCLCPP_ERROR(this->get_logger(), "Expected 8 values [start_x,start_y,start_z,r,p,y,distance,steps], got %zu", 
                   msg->data.size());
      return;
    }
    
    geometry_msgs::msg::Pose start_pose = createPoseFromArray(msg->data, 0);
    double distance = msg->data[6];
    int steps = static_cast<int>(msg->data[7]);
    
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.x += distance; // Sweep in X direction
    
    RCLCPP_INFO(this->get_logger(), "Starting horizontal sweep: %.3fm over %d steps", distance, steps);
    executeCartesianPath(start_pose, end_pose, steps);
  }

  // Vertical sweep
  void sweepVerticalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 8) {
      RCLCPP_ERROR(this->get_logger(), "Expected 8 values [start_x,start_y,start_z,r,p,y,distance,steps], got %zu", 
                   msg->data.size());
      return;
    }
    
    geometry_msgs::msg::Pose start_pose = createPoseFromArray(msg->data, 0);
    double distance = msg->data[6];
    int steps = static_cast<int>(msg->data[7]);
    
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.z += distance; // Sweep in Z direction
    
    RCLCPP_INFO(this->get_logger(), "Starting vertical sweep: %.3fm over %d steps", distance, steps);
    executeCartesianPath(start_pose, end_pose, steps);
  }

  // Grid scan pattern
  void gridScanCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    
    if (msg->data.size() != 10) {
      RCLCPP_ERROR(this->get_logger(), "Expected 10 values [center_x,center_y,center_z,r,p,y,width,height,rows,cols], got %zu", 
                   msg->data.size());
      return;
    }
    
    geometry_msgs::msg::Pose center_pose = createPoseFromArray(msg->data, 0);
    double width = msg->data[6];
    double height = msg->data[7];
    int rows = static_cast<int>(msg->data[8]);
    int cols = static_cast<int>(msg->data[9]);
    
    RCLCPP_INFO(this->get_logger(), "Starting grid scan: %.3f x %.3f, %d rows x %d cols", 
                width, height, rows, cols);
    executeGridScan(center_pose, width, height, rows, cols);
  }

  // Helper function to create pose from array
  geometry_msgs::msg::Pose createPoseFromArray(const std::vector<double>& data, int offset)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = data[offset + 0];
    pose.position.y = data[offset + 1];
    pose.position.z = data[offset + 2];
    
    tf2::Quaternion q;
    q.setRPY(data[offset + 3], data[offset + 4], data[offset + 5]);
    q.normalize();
    pose.orientation = tf2::toMsg(q);
    
    return pose;
  }

  // Execute single pose (your existing logic)
  void executeSinglePose(const geometry_msgs::msg::Pose& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Planning single pose motion...");
    
    arm_group_interface_->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = static_cast<bool>(arm_group_interface_->plan(plan));
    if (success) {
      RCLCPP_INFO(this->get_logger(), "✅ Single pose planning successful! Executing...");
      arm_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Single pose planning failed!");
    }
  }

  // Execute Cartesian path
  void executeCartesianPath(const geometry_msgs::msg::Pose& start_pose, 
                           const geometry_msgs::msg::Pose& end_pose, 
                           int steps)
  {
    RCLCPP_INFO(this->get_logger(), "Planning Cartesian path with %d steps...", steps);
    
    // Generate waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i <= steps; i++) {
      double t = static_cast<double>(i) / static_cast<double>(steps);
      
      geometry_msgs::msg::Pose waypoint;
      waypoint.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x);
      waypoint.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y);
      waypoint.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z);
      waypoint.orientation = start_pose.orientation; // Keep orientation constant
      
      waypoints.push_back(waypoint);
    }
    
    // Plan Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // Disable jump threshold
    const double eef_step = 0.01;       // 1cm resolution
    
    double fraction = arm_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction >= 0.95) { // 95% of path achieved
      RCLCPP_INFO(this->get_logger(), "✅ Cartesian path planning successful (%.1f%%)! Executing...", fraction * 100);
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      arm_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Cartesian path planning failed! Only %.1f%% achieved", fraction * 100);
    }
  }

  // Execute grid scan pattern
  void executeGridScan(const geometry_msgs::msg::Pose& center_pose, 
                      double width, double height, 
                      int rows, int cols)
  {
    double start_x = center_pose.position.x - width / 2.0;
    double start_y = center_pose.position.y - height / 2.0;
    double step_x = width / (cols - 1);
    double step_y = height / (rows - 1);
    
    for (int row = 0; row < rows; row++) {
      for (int col = 0; col < cols; col++) {
        geometry_msgs::msg::Pose scan_pose = center_pose;
        
        // Snake pattern (alternate direction each row)
        int actual_col = (row % 2 == 0) ? col : (cols - 1 - col);
        
        scan_pose.position.x = start_x + actual_col * step_x;
        scan_pose.position.y = start_y + row * step_y;
        
        RCLCPP_INFO(this->get_logger(), "Grid point [%d,%d]: [%.3f, %.3f, %.3f]", 
                    row, actual_col, scan_pose.position.x, scan_pose.position.y, scan_pose.position.z);
        
        executeSinglePose(scan_pose);
        
        // Small delay for processing (ArUco detection, etc.)
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Grid scan completed!");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create node with proper shared_ptr handling
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<CartesianPlanner>();
  
  // Small delay to ensure node is fully initialized
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(node->get_logger(), "🚀 Cartesian Planner ready!");
  
  // Use MultiThreadedExecutor for better performance
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
