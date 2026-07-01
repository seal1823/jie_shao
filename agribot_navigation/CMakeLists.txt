cmake_minimum_required(VERSION 3.8)
project(agribot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(send_goal src/send_goal.cpp)
ament_target_dependencies(send_goal
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(send_goal ${rclcpp_LIBRARIES})

install(TARGETS send_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()