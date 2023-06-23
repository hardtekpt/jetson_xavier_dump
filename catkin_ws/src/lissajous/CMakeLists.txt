cmake_minimum_required(VERSION 2.8.3)
project(lissajous)

find_package(catkin 
 # ROS components
 REQUIRED COMPONENTS
 roscpp
 mavros_cpp
 drone_gimmicks_library
 drone_utils_cpp
 
)

catkin_package(
 CATKIN_DEPENDS
 roscpp
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