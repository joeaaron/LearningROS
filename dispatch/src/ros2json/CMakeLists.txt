cmake_minimum_required(VERSION 2.8.3)
project(ros2json)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories("${JSONCPP_INCLUDE_DIRS}")
#link_libraries(${JSONCPP_LIBRARIES})



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ClientNode  src/client_node.cpp)
target_link_libraries(ClientNode 
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    )


