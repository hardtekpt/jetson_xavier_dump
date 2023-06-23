cmake_minimum_required(VERSION 2.8.3)
project(drone_gimmicks_library)

find_package(catkin REQUIRED
 COMPONENTS
 roscpp
 tf2
 tf2_ros
)

catkin_package(
 INCLUDE_DIRS 
   include
 LIBRARIES
  ${PROJECT_NAME}
 CATKIN_DEPENDS
  roscpp
)

add_compile_options(-std=c++17) 

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/DroneGimmicks.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
