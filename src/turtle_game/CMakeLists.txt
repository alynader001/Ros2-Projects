cmake_minimum_required(VERSION 3.8)
project(turtle_game)

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
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtlesim_spawner src/turtlesim_spawner.cpp)
ament_target_dependencies(turtlesim_spawner rclcpp turtlesim my_robot_interfaces)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS
  turtlesim_spawner
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
