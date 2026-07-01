cmake_minimum_required(VERSION 3.8)
project(agribot_local_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

add_library(agribot_local_planner SHARED
  src/agribot_local_planner.cpp
)

ament_target_dependencies(agribot_local_planner
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_costmap_2d
  nav2_core
  pluginlib
)

target_link_libraries(agribot_local_planner)

install(TARGETS agribot_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES agribot_local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()