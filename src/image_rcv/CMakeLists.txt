cmake_minimum_required(VERSION 3.0.2)
project(image_rcv)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  
)
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES image_rcv
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
   DEPENDS system_lib
)

add_executable(image_rcv_node src/image_rcv_node.cpp )

target_link_libraries(image_rcv_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

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



