cmake_minimum_required(VERSION 3.0.2)
project(bebop_run)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)




find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES bebop_run
   CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs tf geometry_msgs
   DEPENDS system_lib
)

add_executable(bebop_run src/bebop_run.cpp )

target_link_libraries(bebop_run ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
