# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hellorobot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ihellorobot:/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hellorobot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_custom_target(_hellorobot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hellorobot" "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hellorobot
  "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hellorobot
)

### Generating Services

### Generating Module File
_generate_module_cpp(hellorobot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hellorobot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hellorobot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hellorobot_generate_messages hellorobot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_dependencies(hellorobot_generate_messages_cpp _hellorobot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hellorobot_gencpp)
add_dependencies(hellorobot_gencpp hellorobot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hellorobot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hellorobot
  "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hellorobot
)

### Generating Services

### Generating Module File
_generate_module_eus(hellorobot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hellorobot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hellorobot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hellorobot_generate_messages hellorobot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_dependencies(hellorobot_generate_messages_eus _hellorobot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hellorobot_geneus)
add_dependencies(hellorobot_geneus hellorobot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hellorobot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hellorobot
  "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hellorobot
)

### Generating Services

### Generating Module File
_generate_module_lisp(hellorobot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hellorobot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hellorobot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hellorobot_generate_messages hellorobot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_dependencies(hellorobot_generate_messages_lisp _hellorobot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hellorobot_genlisp)
add_dependencies(hellorobot_genlisp hellorobot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hellorobot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hellorobot
  "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hellorobot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hellorobot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hellorobot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hellorobot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hellorobot_generate_messages hellorobot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_dependencies(hellorobot_generate_messages_nodejs _hellorobot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hellorobot_gennodejs)
add_dependencies(hellorobot_gennodejs hellorobot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hellorobot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hellorobot
  "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hellorobot
)

### Generating Services

### Generating Module File
_generate_module_py(hellorobot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hellorobot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hellorobot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hellorobot_generate_messages hellorobot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/JA/code/LearningROS/ros_org/src/hellorobot/msg/robotMsg.msg" NAME_WE)
add_dependencies(hellorobot_generate_messages_py _hellorobot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hellorobot_genpy)
add_dependencies(hellorobot_genpy hellorobot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hellorobot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hellorobot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hellorobot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hellorobot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hellorobot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hellorobot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hellorobot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hellorobot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hellorobot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hellorobot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hellorobot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hellorobot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hellorobot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hellorobot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hellorobot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hellorobot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hellorobot_generate_messages_py std_msgs_generate_messages_py)
endif()
