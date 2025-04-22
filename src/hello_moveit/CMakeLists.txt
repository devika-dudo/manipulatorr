cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

# Function to add executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_link_libraries(${target_name}
    ${rclcpp_LIBRARIES}
    ${moveit_ros_planning_interface_LIBRARIES}
    ${moveit_visual_tools_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES})
  ament_target_dependencies(${target_name}
    rclcpp
    moveit_ros_planning_interface
    moveit_visual_tools
    rviz_visual_tools
    sensor_msgs
    graph_msgs)
  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# Add executables
add_common_executable(hello_moveit letsee.cpp)
add_common_executable(plan_around_objects plan_around_objects.cpp)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

