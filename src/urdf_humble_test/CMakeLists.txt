cmake_minimum_required(VERSION 3.8)
project(urdf_humble_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Uncomment and add any dependencies you need here
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch urdf config meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fake_joint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_joint_publisher
)

install(PROGRAMS
  scripts/pick_and_place_automation.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/autonomous_pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(gazebo_ros REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

