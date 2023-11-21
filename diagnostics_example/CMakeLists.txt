cmake_minimum_required(VERSION 3.5)
project(diagnostics_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(diagnostics_example_node src/diagnostics_example_node.cpp)
ament_target_dependencies(diagnostics_example_node rclcpp)

# Install Cpp executables
install(TARGETS
  diagnostics_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/diagnostics_updater.py
  scripts/aggregator_sub.py
  scripts/computer_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()