# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nautilus_wall_following: 1 messages, 0 services")

set(MSG_I_FLAGS "-Inautilus_wall_following:/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nautilus_wall_following_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_custom_target(_nautilus_wall_following_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nautilus_wall_following" "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nautilus_wall_following
  "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautilus_wall_following
)

### Generating Services

### Generating Module File
_generate_module_cpp(nautilus_wall_following
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautilus_wall_following
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nautilus_wall_following_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nautilus_wall_following_generate_messages nautilus_wall_following_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_dependencies(nautilus_wall_following_generate_messages_cpp _nautilus_wall_following_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautilus_wall_following_gencpp)
add_dependencies(nautilus_wall_following_gencpp nautilus_wall_following_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautilus_wall_following_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nautilus_wall_following
  "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautilus_wall_following
)

### Generating Services

### Generating Module File
_generate_module_eus(nautilus_wall_following
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautilus_wall_following
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nautilus_wall_following_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nautilus_wall_following_generate_messages nautilus_wall_following_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_dependencies(nautilus_wall_following_generate_messages_eus _nautilus_wall_following_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautilus_wall_following_geneus)
add_dependencies(nautilus_wall_following_geneus nautilus_wall_following_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautilus_wall_following_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nautilus_wall_following
  "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautilus_wall_following
)

### Generating Services

### Generating Module File
_generate_module_lisp(nautilus_wall_following
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautilus_wall_following
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nautilus_wall_following_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nautilus_wall_following_generate_messages nautilus_wall_following_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_dependencies(nautilus_wall_following_generate_messages_lisp _nautilus_wall_following_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautilus_wall_following_genlisp)
add_dependencies(nautilus_wall_following_genlisp nautilus_wall_following_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautilus_wall_following_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nautilus_wall_following
  "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautilus_wall_following
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nautilus_wall_following
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautilus_wall_following
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nautilus_wall_following_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nautilus_wall_following_generate_messages nautilus_wall_following_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_dependencies(nautilus_wall_following_generate_messages_nodejs _nautilus_wall_following_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautilus_wall_following_gennodejs)
add_dependencies(nautilus_wall_following_gennodejs nautilus_wall_following_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautilus_wall_following_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nautilus_wall_following
  "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautilus_wall_following
)

### Generating Services

### Generating Module File
_generate_module_py(nautilus_wall_following
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautilus_wall_following
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nautilus_wall_following_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nautilus_wall_following_generate_messages nautilus_wall_following_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/nautilus-F110-2020/wall_follow_ws/src/nautilus_wall_following/msg/error_analysis.msg" NAME_WE)
add_dependencies(nautilus_wall_following_generate_messages_py _nautilus_wall_following_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautilus_wall_following_genpy)
add_dependencies(nautilus_wall_following_genpy nautilus_wall_following_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautilus_wall_following_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautilus_wall_following)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautilus_wall_following
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(nautilus_wall_following_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautilus_wall_following)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautilus_wall_following
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(nautilus_wall_following_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautilus_wall_following)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautilus_wall_following
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(nautilus_wall_following_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautilus_wall_following)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautilus_wall_following
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(nautilus_wall_following_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautilus_wall_following)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautilus_wall_following\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautilus_wall_following
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(nautilus_wall_following_generate_messages_py std_msgs_generate_messages_py)
endif()
