cmake_minimum_required(VERSION 3.8)

project(cpp_pubsub)
set(PCL_DIR "/home/luna/NUC/vcpkg/ports/pcl.cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(serial REQUIRED)
find_package(PCL REQUIRED)
#include_directories(${PCL_DIRS})
#link_directories(${PCL_DIRS})
find_package(pcl_conversions REQUIRED)
find_package(depthimage_to_laserscan REQUIRED)
# find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rviz_rendering REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
"msg/PhoenixControl.msg"
DEPENDENCIES ros_phoenix # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


add_library(cpp_pubsub_tool_plugin SHARED
  src/resetButton.cpp
)
ament_target_dependencies(cpp_pubsub_tool_plugin
  rclcpp
  std_msgs
  rviz_common
)
target_include_directories(cpp_pubsub_tool_plugin PRIVATE
  include
)
ament_target_dependencies(cpp_pubsub_tool_plugin
  rclcpp
  std_msgs
  rviz_common
  rviz_rendering
)
install(TARGETS cpp_pubsub_tool_plugin
  LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(rviz_common cpp_pubsub_plugin_description.xml)
install(FILES
  cpp_pubsub_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)


# add_executable(serial_com src/arduino.cpp)
# ament_target_dependencies(serial_com rclcpp std_msgs geometry_msgs sensor_msgs serial)
# target_link_libraries(serial_com serial)
  
# install(TARGETS
#   serial_comdepthimage_to_laserscan
#   DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/my_sbr.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(talker src/my_pub.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs sensor_msgs ros_phoenix)
  
install(TARGETS
  talker    
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(commander src/pubsub_node.cpp)
ament_target_dependencies(commander rclcpp sensor_msgs geometry_msgs serial)
target_link_libraries(commander serial)

install(TARGETS
  commander
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(rover src/rover.cpp)
ament_target_dependencies(rover rclcpp sensor_msgs geometry_msgs serial)
target_link_libraries(rover serial)

install(TARGETS
  rover
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(motors src/just_motors.cpp)
ament_target_dependencies(motors rclcpp sensor_msgs ros_phoenix serial) 
target_link_libraries(motors serial)

install(TARGETS
  motors
  DESTINATION lib/${PROJECT_NAME})

# New for depthCamera_proccess
add_executable(depthProc src/depthCamera_proccess.cpp)
ament_target_dependencies(depthProc rclcpp sensor_msgs geometry_msgs pcl_conversions PCL depthimage_to_laserscan)
target_link_libraries(depthProc )

install(TARGETS
  depthProc
  DESTINATION lib/${PROJECT_NAME})

# New for combine Point clouds
add_executable(pointcloud_combiner src/combineForScan.cpp)
ament_target_dependencies(pointcloud_combiner rclcpp sensor_msgs std_msgs pcl_conversions PCL)
target_link_libraries(pointcloud_combiner ${Boost_SYSTEM_LIBRARY})

install(TARGETS
  pointcloud_combiner
  DESTINATION lib/${PROJECT_NAME})

# New for reading RVIZ
add_executable(goalHandle src/handleGoals.cpp)
  ament_target_dependencies(goalHandle rclcpp sensor_msgs geometry_msgs nav2_msgs std_msgs)
  target_link_libraries(goalHandle ${Boost_SYSTEM_LIBRARY})

install(TARGETS
  goalHandle
  DESTINATION lib/${PROJECT_NAME})

add_executable(today src/today.cpp)
ament_target_dependencies(today rclcpp sensor_msgs ros_phoenix serial) 
target_link_libraries(today serial)

install(TARGETS
  today
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
