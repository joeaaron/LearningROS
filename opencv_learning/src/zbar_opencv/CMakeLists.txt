cmake_minimum_required(VERSION 2.8.3)
project(zbar_opencv)
set(CMAKE_MODULE_PATH ${ZBARCV_SOURCE_DIR})
find_package (OpenCV)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge##ros的数据转化成opencv数据,再用opencv处理
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
 
find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})
find_library(ZBAR_LIBRARIES NAMES zbar
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS} )
find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZBAR  DEFAULT_MSG  ZBAR_LIBRARIES ZBAR_INCLUDE_DIR)
catkin_package(
 INCLUDE_DIRS include
  LIBRARIES zbar_opencv
)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)
 
link_directories(libs) 

add_executable(zbar_opencv 
      src/zbar_opencv.cpp
      src/zbarScanner.cpp

      include/zbarScanner.h
)

target_link_libraries(zbar_opencv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
 # ${Zbar_LIBRARIES}
  libzbar.so##最重要的添加编译用的共享库
 
)
