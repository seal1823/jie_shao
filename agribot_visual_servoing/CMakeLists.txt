cmake_minimum_required(VERSION 3.8)
project(agribot_visual_servoing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(crop_row_detector_node
  src/crop_row_detector.cpp
  src/visual_servoing_node.cpp
)
ament_target_dependencies(crop_row_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
)
target_link_libraries(crop_row_detector_node
  ${OpenCV_LIBS}
)

install(TARGETS crop_row_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()