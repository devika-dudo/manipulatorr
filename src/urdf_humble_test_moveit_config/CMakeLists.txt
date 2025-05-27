cmake_minimum_required(VERSION 3.22)
project(urdf_humble_test_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_and_place_node scripts/pick_and_place.cpp)
add_executable(joystick_cartesian_teleop_node scripts/joystick_cartesian_teleop.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_common
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)
ament_target_dependencies(joystick_cartesian_teleop_node
  rclcpp
  moveit_common
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

install(TARGETS
  joystick_cartesian_teleop_node
  pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

ament_package()

