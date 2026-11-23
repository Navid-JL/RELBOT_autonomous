cmake_minimum_required(VERSION 3.16)
project(relbot_hardware_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core hardware communication library (compiles with or without ROS 2 installed)
add_library(relbot_hw_core STATIC
  src/serial_mcu_bridge.cpp
  src/relbot_base_driver.cpp
  src/manipulator_sequencer.cpp
)
target_include_directories(relbot_hw_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Standalone hardware verification / unit test executable
enable_testing()
add_executable(test_hw_core test/test_hw_core.cpp)
target_link_libraries(test_hw_core PRIVATE relbot_hw_core)
add_test(NAME test_hw_core COMMAND test_hw_core)

# Optional ROS 2 node build when ament_cmake & rclcpp are available in the environment
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(relbot_interfaces QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND relbot_interfaces_FOUND)
  message(STATUS "ROS 2 environment detected: building relbot_hardware_node")
  add_executable(relbot_hardware_node src/relbot_hardware_node.cpp)
  target_link_libraries(relbot_hardware_node relbot_hw_core)
  ament_target_dependencies(relbot_hardware_node
    rclcpp
    geometry_msgs
    nav_msgs
    relbot_interfaces
  )

  install(TARGETS
    relbot_hw_core
    relbot_hardware_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
else()
  message(STATUS "Building standalone C++ hardware library & tests (ROS 2 not sourced)")
endif()
