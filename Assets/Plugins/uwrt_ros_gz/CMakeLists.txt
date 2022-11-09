cmake_minimum_required(VERSION 3.8)
project(uwrt_ros_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)

#add bridge to ros2 
add_executable(uwrt_ros_gz_bridge src/bridge.cpp)

ament_target_dependencies(uwrt_ros_gz_bridge
  rclcpp
  geometry_msgs
  std_msgs 
  gz-sim7
)

#install
install(TARGETS
  uwrt_ros_gz_bridge
  DESTINATION lib/${PROJECT_NAME})

#install for launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
