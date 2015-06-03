# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${state_DIR}/.." "msg" state_MSG_INCLUDE_DIRS UNIQUE)
set(state_MSG_DEPENDENCIES std_msgs;geometry_msgs;sensor_msgs)
