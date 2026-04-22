cmake_minimum_required(VERSION 3.8)
project(line_tracking_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/line_follower_node.py
  scripts/obstacle_avoidance_node.py
  scripts/camera_processor_node.py
  scripts/robot_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
