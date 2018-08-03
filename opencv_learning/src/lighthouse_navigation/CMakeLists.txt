cmake_minimum_required(VERSION 2.8.3)
project(lighthouse_navigation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fPIC")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs tf genmsg dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
   cfg/LighthouseRecognize.cfg
)

include_directories(${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
include
/usr/include/eigen3)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  message_runtime
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#file(GLOB HEADER_FILES include/*.h)

include_directories(include)

link_directories(${EIGEN3_LIBRARY_DIRS})

add_executable(lighthouse_recognize src/lighthouse_recognize.cpp)
target_link_libraries(lighthouse_recognize ${catkin_LIBRARIES})
add_dependencies(lighthouse_recognize
        ${PROJECT_NAME}_generate_messages_cpp
        ${artoolkit_EXPORTED_TARGETS})

add_executable(lighthouse_planner src/lighthouse_planner.cpp)
target_link_libraries(lighthouse_planner ${catkin_LIBRARIES}
/opt/ros/indigo/lib/libeigen_conversions.so)#未添加此行tf::poseMsgToEigen编译不过
add_dependencies(lighthouse_planner
        ${PROJECT_NAME}_generate_messages_cpp
        ${artoolkit_EXPORTED_TARGETS})

#add_executable(triangle_recognize src/triangle_recognize.cpp)
#target_link_libraries(triangle_recognize ${catkin_LIBRARIES})
#add_dependencies(triangle_recognize
#        ${PROJECT_NAME}_generate_messages_cpp
#        ${artoolkit_EXPORTED_TARGETS})

#add_executable(reflector_recognize src/reflector_recognize.cpp)
#target_link_libraries(reflector_recognize ${catkin_LIBRARIES})# ${CERES_LIBRARIES})
#add_dependencies(reflector_recognize
#        ${PROJECT_NAME}_generate_messages_cpp
#        ${artoolkit_EXPORTED_TARGETS})

#add_executable(recognize_precision test/recognize_precision.cpp)
#target_link_libraries(recognize_precision ${catkin_LIBRARIES})
#add_dependencies(recognize_precision
#        ${PROJECT_NAME}_generate_messages_cpp
#        ${artoolkit_EXPORTED_TARGETS})

add_executable(magnetic_driver src/magnetic_driver.cpp ${HEADER_FILES})
target_link_libraries(magnetic_driver ${catkin_LIBRARIES})
add_dependencies(magnetic_driver ${PROJECT_NAME}_generate_messages_cpp)

add_executable(magnetic_tf_pub src/magnetic_tf_pub.cpp)
target_link_libraries(magnetic_tf_pub ${catkin_LIBRARIES})# ${CERES_LIBRARIES})
add_dependencies(magnetic_tf_pub
        ${PROJECT_NAME}_generate_messages_cpp
        ${artoolkit_EXPORTED_TARGETS})
