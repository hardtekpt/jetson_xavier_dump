cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 message_generation
 check_interest_points
 waypoint_following
 # messages
 std_msgs 
 geometry_msgs
 mavros_msgs
 mavros_cpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 message_runtime
 std_msgs
 geometry_msgs
 mavros_msgs
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${JSONCPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  # ROS specific details 
  src/ros/main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
