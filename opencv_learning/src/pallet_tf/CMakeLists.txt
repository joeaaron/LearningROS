cmake_minimum_required(VERSION 2.8.3)
project(pallet_tf)

find_package (OpenCV)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs tf dynamic_reconfigure)
## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/param.cfg
)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(pallet_tf 
      src/pallet_tf.cpp
      src/pallet_tf_node.cpp
      include/pallet_tf/pallet_tf.h
)

target_link_libraries(pallet_tf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

