cmake_minimum_required(VERSION 2.8.3)
project(quad_navig)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quad_navig
  # CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(quad_navig
  src/quad_navig.cpp
  src/quadScanner.cpp
)

target_link_libraries(quad_navig
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
