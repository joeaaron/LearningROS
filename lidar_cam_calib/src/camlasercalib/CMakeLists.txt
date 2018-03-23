cmake_minimum_required(VERSION 2.8.3)
project(camlasercalib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp std_msgs dynamic_reconfigure cv_bridge)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS signals)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/pointcloudcut.cfg
)



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
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(lidar_cam 
		src/camlasercalib_node.cpp
    src/camLaserCalib.cpp
    src/pcl_util.cpp
    src/Marker.cpp
    src/MarkerDetector.cpp
    
    include/camLaserCalib.h
    include/pcl_util.h
    include/Marker.h
    include/MarkerDetector.h
)


## Specify libraries to link a library or executable target against
target_link_libraries(lidar_cam
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
 )

