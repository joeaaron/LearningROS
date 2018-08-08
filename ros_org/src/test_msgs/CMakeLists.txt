cmake_minimum_required(VERSION 2.8.3)
project(test_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Test.msg
)

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_msgs
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

add_executable(talker src/talker.cpp)
add_dependencies(talker test_msgs_gencpp)#调用同一工作空间的自定义消息类型时注明依赖关系，防止发生头文件找不到的报错
target_link_libraries(talker ${catkin_LIBRARIES})

