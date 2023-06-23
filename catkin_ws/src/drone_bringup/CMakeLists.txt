cmake_minimum_required(VERSION 3.0.2)
project(drone_bringup)

find_package(catkin 
  REQUIRED COMPONENTS
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
