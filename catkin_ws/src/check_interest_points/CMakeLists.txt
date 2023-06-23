cmake_minimum_required(VERSION 2.8.3)
project(check_interest_points)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 mavros_cpp
 drone_gimmicks_library
 drone_utils_cpp
 message_generation

  # messages
  std_msgs 
  geometry_msgs
  visualization_msgs
  sensor_msgs
  mavros_msgs
)

add_service_files(
  FILES
  StartMission.srv
)

add_message_files(
  FILES
  DebugMSG.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 message_runtime
 geometry_msgs
 visualization_msgs
)
 

find_package(
  # Eigen Library required
  Eigen3 REQUIRED  
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 # Eigen Library is required for this package
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
  # ROS specific details 
  src/ros/main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
