cmake_minimum_required(VERSION 3.22)
project(urdf_humble_test_moveit_config)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find GLUT - needed for MoveIt visualization components
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

# Add the C++ executable
add_executable(arm_control_from_UI scripts/arm_control_from_UI.cpp)
target_include_directories(arm_control_from_UI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(arm_control_from_UI PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  arm_control_from_UI
  rclcpp
  rclcpp_components
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link against GLUT and OpenGL explicitly
target_link_libraries(arm_control_from_UI
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

# Install the C++ executable
install(TARGETS
  arm_control_from_UI
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts if they exist
install(PROGRAMS
  scripts/arm_control_from_UI.cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
