cmake_minimum_required(VERSION 3.0.2)
project(off2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  tf2
  
)



catkin_package(
 INCLUDE_DIRS include
 LIBRARIES off2
 CATKIN_DEPENDS geometry_msgs mavros roscpp
 DEPENDS system_lib
)

add_executable(off2_node src/off2_node.cpp )

target_link_libraries(off2_node ${catkin_LIBRARIES})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Testing ##
#############


