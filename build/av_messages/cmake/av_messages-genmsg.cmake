# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "av_messages: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iav_messages:/home/sahil/WheelChair-Codes/src/av_messages/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(av_messages_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_custom_target(_av_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "av_messages" "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" "geometry_msgs/Pose2D:std_msgs/Int16:geometry_msgs/Point:std_msgs/String"
)

get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_custom_target(_av_messages_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "av_messages" "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" "std_msgs/Int16:geometry_msgs/Point:av_messages/object_:geometry_msgs/Pose2D:std_msgs/Header:std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/av_messages
)
_generate_msg_cpp(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/av_messages
)

### Generating Services

### Generating Module File
_generate_module_cpp(av_messages
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/av_messages
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(av_messages_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(av_messages_generate_messages av_messages_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_cpp _av_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_cpp _av_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(av_messages_gencpp)
add_dependencies(av_messages_gencpp av_messages_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS av_messages_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/av_messages
)
_generate_msg_eus(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/av_messages
)

### Generating Services

### Generating Module File
_generate_module_eus(av_messages
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/av_messages
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(av_messages_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(av_messages_generate_messages av_messages_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_eus _av_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_eus _av_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(av_messages_geneus)
add_dependencies(av_messages_geneus av_messages_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS av_messages_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/av_messages
)
_generate_msg_lisp(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/av_messages
)

### Generating Services

### Generating Module File
_generate_module_lisp(av_messages
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/av_messages
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(av_messages_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(av_messages_generate_messages av_messages_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_lisp _av_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_lisp _av_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(av_messages_genlisp)
add_dependencies(av_messages_genlisp av_messages_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS av_messages_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/av_messages
)
_generate_msg_nodejs(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/av_messages
)

### Generating Services

### Generating Module File
_generate_module_nodejs(av_messages
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/av_messages
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(av_messages_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(av_messages_generate_messages av_messages_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_nodejs _av_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_nodejs _av_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(av_messages_gennodejs)
add_dependencies(av_messages_gennodejs av_messages_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS av_messages_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages
)
_generate_msg_py(av_messages
  "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Int16.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages
)

### Generating Services

### Generating Module File
_generate_module_py(av_messages
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(av_messages_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(av_messages_generate_messages av_messages_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/object_.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_py _av_messages_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sahil/WheelChair-Codes/src/av_messages/msg/objects.msg" NAME_WE)
add_dependencies(av_messages_generate_messages_py _av_messages_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(av_messages_genpy)
add_dependencies(av_messages_genpy av_messages_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS av_messages_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/av_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/av_messages
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(av_messages_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(av_messages_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(av_messages_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(av_messages_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/av_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/av_messages
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(av_messages_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(av_messages_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(av_messages_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(av_messages_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/av_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/av_messages
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(av_messages_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(av_messages_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(av_messages_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(av_messages_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/av_messages)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/av_messages
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(av_messages_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(av_messages_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(av_messages_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(av_messages_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/av_messages
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(av_messages_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(av_messages_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(av_messages_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(av_messages_generate_messages_py nav_msgs_generate_messages_py)
endif()
