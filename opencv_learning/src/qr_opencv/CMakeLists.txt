cmake_minimum_required(VERSION 2.8.3)
project(qr_opencv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

find_package(OpenCV REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
# ${catkin_INCLUDE_DIRS}
#)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(qr_cam_tf
	  src/qrcam.cpp
    src/findMarker.cpp
    
    include/findMarker.h
)

## Specify libraries to link a library or executable target against
target_link_libraries(qr_cam_tf
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
 )

