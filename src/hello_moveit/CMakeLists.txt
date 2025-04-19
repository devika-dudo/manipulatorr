cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

# Add executable
add_executable(hello_moveit src/letsee.cpp)

# Include directories
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Link libraries
target_link_libraries(hello_moveit
  ${rclcpp_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)

# Define C++ standard
target_compile_features(hello_moveit PUBLIC cxx_std_17)

# Set ament target dependencies
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  sensor_msgs
  graph_msgs
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install targets
install(TARGETS
  hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

# Testing configuration (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

