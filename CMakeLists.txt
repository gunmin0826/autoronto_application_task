cmake_minimum_required(VERSION 3.8)
project(autoronto_app_task_ima)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(talker src/test_pub.cpp)
add_executable(listener src/test_sub.cpp)
adD_executable(solver src/solution.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(solver rclcpp std_msgs message_filters)

install(TARGETS
  talker
  listener
  solver
  DESTINATION lib/${PROJECT_NAME})


ament_package()
