cmake_minimum_required(VERSION 2.8.3)
project(rviz_uav_marker)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 message_generation
 drone_utils_cpp
 # messages
 std_msgs 
 geometry_msgs
 mavros_msgs
 visualization_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 message_runtime
 std_msgs
 geometry_msgs
 mavros_msgs
 visualization_msgs
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  # ROS specific details 
  src/ros/main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})