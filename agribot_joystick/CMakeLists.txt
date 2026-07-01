cmake_minimum_required(VERSION 3.8)
project(agribot_joystick)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(agribot_joystick src/agribot_joystick.cpp)
target_link_libraries(agribot_joystick ${rclcpp_LIBRARIES})
ament_target_dependencies(agribot_joystick
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS agribot_joystick
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()