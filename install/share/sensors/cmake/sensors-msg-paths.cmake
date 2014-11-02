# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${sensors_DIR}/.." "msg" sensors_MSG_INCLUDE_DIRS UNIQUE)
set(sensors_MSG_DEPENDENCIES std_msgs;geometry_msgs;sensor_msgs)
