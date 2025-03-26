cmake_minimum_required(VERSION 3.5)
project(grasp01_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp_action REQUIRED)

# Debug Pinocchio paths
if(pinocchio_FOUND)
  message(STATUS "Found Pinocchio: ${pinocchio_INCLUDE_DIRS}")
  message(STATUS "Pinocchio libraries: ${pinocchio_LIBRARIES}")
else()
  message(FATAL_ERROR "Pinocchio not found. Please install ros-humble-pinocchio.")
endif()

# Define the library target
add_library(
  ${PROJECT_NAME}
  SHARED
  src/grasp01_impedance_control.cpp
)

# Include directories
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}  # Ensure Pinocchio headers are included
)

# Link dependencies
ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  hardware_interface
  rclcpp_action
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  moveit_ros_planning
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${moveit_ros_planning_LIBRARIES}
  pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(
  controller_interface grasp01_controller.xml
)

# Install rules
install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export settings
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  moveit_core
  moveit_ros_planning
)

ament_package()