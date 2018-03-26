cmake_minimum_required(VERSION 2.8.3)
project(basic_usage)

SET( CMAKE_BUILD_TYPE Debug )
SET( CMAKE_CXX_COMPILER "g++" )
SET( CMAKE_CXX_FLAGS "-std=c++0x" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})


add_executable(MatrixTransform src/matrix_transform.cpp)
target_link_libraries(MatrixTransform ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})