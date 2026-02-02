# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "distributed_pgo: 2 messages, 0 services")

set(MSG_I_FLAGS "-Idistributed_pgo:/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(distributed_pgo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_custom_target(_distributed_pgo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "distributed_pgo" "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" ""
)

get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_custom_target(_distributed_pgo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "distributed_pgo" "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/distributed_pgo
)
_generate_msg_cpp(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/distributed_pgo
)

### Generating Services

### Generating Module File
_generate_module_cpp(distributed_pgo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/distributed_pgo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(distributed_pgo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(distributed_pgo_generate_messages distributed_pgo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_cpp _distributed_pgo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_cpp _distributed_pgo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(distributed_pgo_gencpp)
add_dependencies(distributed_pgo_gencpp distributed_pgo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS distributed_pgo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/distributed_pgo
)
_generate_msg_eus(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/distributed_pgo
)

### Generating Services

### Generating Module File
_generate_module_eus(distributed_pgo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/distributed_pgo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(distributed_pgo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(distributed_pgo_generate_messages distributed_pgo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_eus _distributed_pgo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_eus _distributed_pgo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(distributed_pgo_geneus)
add_dependencies(distributed_pgo_geneus distributed_pgo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS distributed_pgo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/distributed_pgo
)
_generate_msg_lisp(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/distributed_pgo
)

### Generating Services

### Generating Module File
_generate_module_lisp(distributed_pgo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/distributed_pgo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(distributed_pgo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(distributed_pgo_generate_messages distributed_pgo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_lisp _distributed_pgo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_lisp _distributed_pgo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(distributed_pgo_genlisp)
add_dependencies(distributed_pgo_genlisp distributed_pgo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS distributed_pgo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/distributed_pgo
)
_generate_msg_nodejs(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/distributed_pgo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(distributed_pgo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/distributed_pgo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(distributed_pgo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(distributed_pgo_generate_messages distributed_pgo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_nodejs _distributed_pgo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_nodejs _distributed_pgo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(distributed_pgo_gennodejs)
add_dependencies(distributed_pgo_gennodejs distributed_pgo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS distributed_pgo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo
)
_generate_msg_py(distributed_pgo
  "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo
)

### Generating Services

### Generating Module File
_generate_module_py(distributed_pgo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(distributed_pgo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(distributed_pgo_generate_messages distributed_pgo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoUpdate.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_py _distributed_pgo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hsuanpin/dpgo_2D_ws/src/DPGO_PUDQ_C-_ROS-main/msg/PgoStatus.msg" NAME_WE)
add_dependencies(distributed_pgo_generate_messages_py _distributed_pgo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(distributed_pgo_genpy)
add_dependencies(distributed_pgo_genpy distributed_pgo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS distributed_pgo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/distributed_pgo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/distributed_pgo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(distributed_pgo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/distributed_pgo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/distributed_pgo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(distributed_pgo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/distributed_pgo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/distributed_pgo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(distributed_pgo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/distributed_pgo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/distributed_pgo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(distributed_pgo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/distributed_pgo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(distributed_pgo_generate_messages_py std_msgs_generate_messages_py)
endif()
