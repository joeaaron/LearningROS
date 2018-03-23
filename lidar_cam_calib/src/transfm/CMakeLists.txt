CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(PROJECT_NAME solvepnp)
PROJECT(${PROJECT_NAME})
SET( CMAKE_BUILD_TYPE Debug )
SET( CMAKE_CXX_COMPILER "g++" )
SET( CMAKE_CXX_FLAGS "-std=c++0x" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add OpenCV library
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
	#MESSAGE(STATUS "\n ~~~~~~~~OpenCV_FOUND~~~~~~~~~~\n")
	include_directories(${OpenCV_INCLUDE_DIRS})
	# message("OpenCV_INCLUDE_DIRS : "  ${OpenCV_INCLUDE_DIRS}"\n" )
         message("OpenCV_VERSION : "  ${OpenCV_VERSION} \n )
	link_directories(${OpenCV_LIBRARY_DIRS})
endif()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(solvepnp 
	src/solvepnp.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(solvepnp
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
 )

