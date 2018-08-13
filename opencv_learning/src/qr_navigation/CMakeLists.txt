cmake_minimum_required(VERSION 2.8.3)
project(qr_navigation)
set(CMAKE_MODULE_PATH ${ZBARCV_SOURCE_DIR})
find_package (OpenCV)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge##ros的数据转化成opencv数据,再用opencv处理
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

## Generate message in the 'msg' folder
add_message_files(
    FILES
    qrMsg.msg
    )
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
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
  LIBRARIES qr_navig
  CATKIN_DEPENDS message_runtime
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
 
add_executable(qr_navig 
      src/qr_navig.cpp
      src/barScanner.cpp

      include/barScanner.h
)

target_link_libraries(qr_navig
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
 # ${Zbar_LIBRARIES}
  /usr/local/lib/libzbar.so##最重要的添加编译用的共享库
 
)
