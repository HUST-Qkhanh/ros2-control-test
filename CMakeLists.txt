cmake_minimum_required(VERSION 3.5)
project(amr_robot_hardware)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wsign-compare)
endif()

# find dependencies
find_package(control_toolbox REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/amr_system.cpp
  src/pid.cpp
)
include_directories(
 include
 include/amr_robot_hardware
 ${Boost_INCLUDE_DIRS}
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  control_toolbox
  rclcpp
  rclcpp_components
)

add_executable(hoverboard_driver 
  src/amr_system.cpp 
  src/pid.cpp
  src/main.cpp
)

target_link_libraries(hoverboard_driver ${Boost_LIBRARIES})
ament_target_dependencies(hoverboard_driver
  rclcpp
  pluginlib
  control_toolbox
  controller_manager
  hardware_interface
)
install(TARGETS hoverboard_driver DESTINATION lib/${PROJECT_NAME})
pluginlib_export_plugin_description_file(hardware_interface amr_robot_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  control_toolbox
  rclcpp
  rclcpp_components
)
ament_package()
