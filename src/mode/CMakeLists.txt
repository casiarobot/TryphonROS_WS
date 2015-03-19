# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(mode)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs genmsg dynamic_reconfigure)

## Declare ROS messages and services

#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services

generate_dynamic_reconfigure_options(
  cfg/mode.cfg
)

## Declare a catkin package
catkin_package( INCLUDE_DIRS
#Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(mode src/mode.cpp)
target_link_libraries(mode ${catkin_LIBRARIES})
add_dependencies(mode std_msgs)

#%EndTag(FULLTEXT)%

