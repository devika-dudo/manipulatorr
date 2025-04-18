cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED) # Add this line
find_package(graph_msgs REQUIRED)

add_executable(hello_moveit src/hello_moveit_v1.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(hello_moveit
  ${rclcpp_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES} # Add this line
)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  hello_moveit
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools # While linked directly, it's good to keep as a dependency
  rviz_visual_tools   # Add this line
  graph_msgs
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(TARGETS
  hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
