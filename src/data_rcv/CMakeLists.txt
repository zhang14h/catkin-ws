cmake_minimum_required(VERSION 3.0.2)
project(data_rcv)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  sensor_msgs
  tf2
  tf
  std_msgs
)

#find_package(OpenCV REQUIRED)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES data_rcv
   CATKIN_DEPENDS geometry_msgs mavros roscpp sensor_msgs #std_msgs
   DEPENDS system_lib
)

add_executable(data_rcv_node src/data_rcv_node.cpp )

target_link_libraries(data_rcv_node ${catkin_LIBRARIES}${OpenCV_LIBRARIES})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

 



## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_data_rcv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
