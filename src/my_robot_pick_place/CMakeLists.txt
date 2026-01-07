cmake_minimum_required(VERSION 3.8)
project(my_robot_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pick_place_node src/pick_place_node.cpp)

ament_target_dependencies(
  pick_place_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
